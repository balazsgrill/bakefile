#
#  This file is part of Bakefile (http://bakefile.org)
#
#  Copyright (C) 2009-2013 Vaclav Slavik
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to
#  deal in the Software without restriction, including without limitation the
#  rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
#  sell copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in
#  all copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
#  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
#  IN THE SOFTWARE.
#
from bkl.plugins.native import LibraryType

"""
GNU tools (GCC, GNU Make, ...) toolset.
"""

import os.path
from bkl.api import TargetType, Property
from bkl.vartypes import *
from bkl.api import FileCompiler, FileType
from bkl.plugins.native import NativeLinkedType
from bkl.makefile import MakefileToolset, MakefileFormatter, MakefileExprFormatter
import bkl.compilers
import bkl.expr
from bkl.compilers import *
from sets import Set

# FIXME: shouldn't be needed later
from bkl.expr import ListExpr, LiteralExpr, BoolExpr, NonConstError, ConcatExpr
from bkl.error import Error

# These are just some unique strings, the exact syntax doesn't matter currently.
GMAKE_IFEXPR_MACROS_PLACEHOLDER = "{{{BKL_GMAKE_IFEXPR_MACROS}}}"
GMAKE_BUILDDIR_DEF_PLACEHOLDER = "{{{BKL_GMAKE_BUILDDIR_DEF}}}"

# GNU Make has some boolean functions, but not all that we need, so define them
GMAKE_IFEXPR_MACROS = """
_true  := true
_false :=
_not    = $(if $(1),$(_false),$(_true_))
_equal  = $(and $(findstring $(1),$(2)),$(findstring $(2),$(1)))

"""


class XC8ObjectFileType(FileType):
    name = "xc8-intermediate-code"
    def __init__(self):
        FileType.__init__(self, extensions=["p1"])

class XC8LibraryFileType(FileType):
    name = "xc8-intermediate-code-library"
    def __init__(self):
        FileType.__init__(self, extensions=["lpp"])
        
class XC8HexFileType(FileType):
    name = "xc8-intel-hex-output"
    def __init__(self):
        FileType.__init__(self, extensions=["hex"])

class XC8FileCompiler(FileCompiler):
    """Base class for XC8 compilers/linkers."""
    def is_supported(self, toolset):
        return isinstance(toolset, XC8Toolset)

    # TODO: a hack, not exactly clean
    def _chip_flags(self, toolset, target):
        return [LiteralExpr("--chip="+target["chip"].value)]


class XC8CCompiler(XC8FileCompiler):
    """
    XC8 C compiler.
    """
    name = "XC8 C"
    in_type = bkl.compilers.CFileType.get()
    out_type = XC8ObjectFileType.get()

    _compiler = "CC"
    _flags_var_name = "CFLAGS"
    _options_prop_name = "c-compiler-options"

    def commands(self, toolset, target, input, output):
        cmd = [LiteralExpr("$(%s) -c -O$@ --output=lpp $(CPPFLAGS) $(%s)" %
                (self._compiler, self._flags_var_name))]
        
        # FIXME: evaluating the flags here every time is inefficient
        cmd += self._chip_flags(toolset, target)
        
        cmd += bkl.expr.add_prefix("-D", target["defines"])
        cmd += bkl.expr.add_prefix("-I", target["includedirs"])
        if target["warnings"] == "no":
            cmd.append(LiteralExpr("-w"))
        elif target["warnings"] == "all":
            cmd.append(LiteralExpr("-Wall"))
        #else: don't do anything special for "minimal" and "default"
        cmd += target["compiler-options"]
        cmd += target[self._options_prop_name]
        
        
        depincludedirs = Set()
        project = target.project
        deps = reversed([project.get_target(x.as_py()) for x in target["deps"]])
        todo = (x for x in deps if isinstance(x.type, XC8LibraryType))
        for x in todo:
            for h in x.headers:
                depincludedirs.add(h.filename.get_directory_path())
            
        for idir in depincludedirs:
            cmd += [ConcatExpr([LiteralExpr("-I"), idir])]
        # FIXME: use a parser instead of constructing the expression manually
        #        in here
        cmd.append(input)
        retval = [ListExpr(cmd)]

        return retval

class XC8Linker(XC8FileCompiler):
    """
    XC8 executables linker.
    """
    name = "XC8 LD"
    in_type = XC8ObjectFileType.get()
    out_type = XC8HexFileType.get()

    def _linker_flags(self, toolset, target):
        cmd = []
        cmd += target.type.get_link_options(target)

        return cmd

    def commands(self, toolset, target, input, output):
        cmd = [LiteralExpr("$(CC) -O$@ $(LDFLAGS)")]
        cmd += self._chip_flags(toolset, target)
        if target["codeoffset"].value != "0":
            cmd += [LiteralExpr("--codeoffset="+target["codeoffset"].value)]
        # FIXME: use a parser instead of constructing the expression manually
        #        in here
        cmd += self._linker_flags(toolset, target)
        cmd += [input]
        for x in target.type.get_linkable_deps(target):
            if isinstance(x.type, XC8LibraryType):
                cmd += [x.type.target_file(toolset, x)]
        return [ListExpr(cmd)]



class XC8LibLinker(XC8FileCompiler):
    """
    GNU library linker.
    """
    name = "XC8 AR"
    in_type = XC8ObjectFileType.get()
    out_type = XC8LibraryFileType.get()

    def commands(self, toolset, target, input, output):
        # FIXME: use a parser instead of constructing the expression manually
        #        in here
        cmd = [LiteralExpr("$(AR) r $@"), input]
        #cmd += self._chip_flags(toolset, target)
        return [ListExpr(cmd)]


class XC8MakefileFormatter(MakefileFormatter):
    """
    Formatter for the GNU Make syntax.
    """
    def var_definition(self, var, value):
        # TODO: use = if it depends on any of the macros defined later
        return "%s ?= %s\n" % (var, " \\\n\t".join(value.split("\n")))

    def submake_command(self, directory, filename, target):
        return "$(MAKE) -C %s -f %s %s" % (directory, filename, target)

    def multifile_target(self, outputs, outfiles, deps, commands):
        # Use a helper intermediate target to handle multiple outputs of a rule,
        # because we can't easily use GNU Make's pattern rules matching. The
        # absence of an intermediate file is not a problem and does not cause
        # spurious builds. See for details:
        #   http://www.gnu.org/software/make/manual/html_node/Chained-Rules.html
        #   http://stackoverflow.com/a/10609434/237188
        for c in commands:
            if '$@' in c:
                raise Error("The use of $@ or %%(out) not supported with multiple outputs (in \"%s\")" % c)
        inter_name = ".dummy_" + "_".join("_".join(c.as_py() for c in f.components) for f in outputs)
        return "\n".join([
            "%s: %s" % (" ".join(outfiles), inter_name),
            ".INTERMEDIATE: %s" % inter_name,
            self.target(inter_name, deps, commands)
            ])


class GnuExprFormatter(MakefileExprFormatter):
    def path(self, e):
        # We handle all build paths in a very special way to allow customizing
        # them at make time by setting the make builddir variable, which is
        # used to initialize another make variable called _builddir which is
        # then used to construct all build paths.
        if e.anchor == bkl.expr.ANCHOR_BUILDDIR:
            # Notice that _builddir is either empty or contains the
            # trailing slash, so we must not add another one here.
            self.toolset.uses_builddir = True
            return "$(_builddir)" + "/".join(self.format(c) for c in e.components)

        super_self = super(GnuExprFormatter, self)

        if e.anchor == bkl.expr.ANCHOR_TOP_BUILDDIR:
            self.toolset.uses_builddir = True

            # To handle top build directory-relative paths correctly, just
            # interpret the path relatively to the top source directory.
            p = super_self.path(bkl.expr.PathExpr(e.components, bkl.expr.ANCHOR_TOP_SRCDIR))

            # But then root it at build directory.
            return "$(_builddir)" + p

        return super_self.path(e)

    def bool_value(self, e):
        self.toolset.uses_non_std_bool_macros = True
        return "$(_true)" if e.value else "$(_false)"

    def bool(self, e):
        l = self.format(e.left)
        if e.right is not None:
            r = self.format(e.right)
        if e.operator == BoolExpr.AND:
            return "$(and %s,%s)" % (l, r)
        if e.operator == BoolExpr.OR:
            return "$(or %s,%s)" % (l, r)
        if e.operator == BoolExpr.EQUAL:
            self.toolset.uses_non_std_bool_macros = True
            return "$(call _equal,%s,%s)" % (l, r)
        if e.operator == BoolExpr.NOT_EQUAL:
            self.toolset.uses_non_std_bool_macros = True
            return "$(call _not,$(call _equal,%s,%s))" % (l, r)
        if e.operator == BoolExpr.NOT:
            self.toolset.uses_non_std_bool_macros = True
            return "$(call _not,%s)" % l
        assert False, "invalid operator"

    def if_(self, e):
        try:
            return super(GnuExprFormatter, self).if_(e)
        except NonConstError:
            c = self.format(e.cond)
            y = self.format(e.value_yes)
            n = self.format(e.value_no)
            return "$(if %s,%s,%s)" % (c, y, n)

class XC8LibraryType(LibraryType):
    """
    Static library.
    """
    name = "xc8-library"

    properties = [
            Property("chip",
                 type=StringType(),
                 inheritable=False,
                 doc="""
                     Target chip
                     """),
        ]

    def get_build_subgraph(self, toolset, target):
        return get_compilation_subgraph(
                        toolset,
                        target,
                        ft_to=XC8LibraryFileType.get(),
                        outfile=self.target_file(toolset, target))

class XC8ApplicationType(NativeLinkedType):
    """
    Executable program.
    """
    name = "xc8-application"

    properties = [
            Property("basename",
                 type=StringType(),
                 default="$(id)",
                 inheritable=False,
                 doc="""
                     Base name of the executable.

                     This is not full filename or even path, it's only its base part,
                     to which platform-specific extension is
                     added. By default, it's the same as target's ID, but it can be changed e.g.
                     if the filename should contain version number, which would be impractical
                     to use as target identifier in the bakefile.

                     .. code-block:: bkl

                        program mytool {
                          // use mytool2.exe or /usr/bin/mytool2
                          basename = $(id)$(vermajor);
                        }
                     """),
            Property("codeoffset",
                 type=StringType(),
                 default="0",
                 inheritable=True,
                 doc="""
                     Specified Code offset argument passed to the compiler
                     """),
        ]

    def get_build_subgraph(self, toolset, target):
        return get_compilation_subgraph(
                        toolset,
                        target,
                        ft_to=XC8HexFileType.get(),
                        outfile=self.target_file(toolset, target))

class XC8Toolset(MakefileToolset):
    """
    GNU toolchain for Unix systems.

    This toolset generates makefiles for the GNU toolchain -- GNU Make, GCC compiler,
    GNU LD linker etc. -- running on Unix system.

    Currently, only Linux systems (or something sufficiently compatible) are supported.
    In particular, file extensions and linker behavior (symlinks, sonames) are assumed
    to be Linux ones.

    See :ref:`ref_toolset_gnu-osx` for OS X variant.
    """
    name = "xc8"

    Formatter = XC8MakefileFormatter
    ExprFormatter = GnuExprFormatter
    default_makefile = "Makefile"

    default_cc = "xc8"

    autoclean_extensions = ["o", "d"]
    del_command = "rm -f"

    object_type = XC8ObjectFileType.get()

    extra_link_flags = None

    def output_default_flags(self, file, configs):
        """
            Helper of on_header() which outputs default, config-dependent,
            values for all the usual compilation flags.
        """

        # Check if we have any custom configurations: we always have at least
        # two standard ones, "Debug" and "Release".
        if len(configs) > 2:
            # We do, so check which of them should use debug settings and
            # which -- the release ones.
            debug_config, release_config = configs['Debug'], configs['Release']
            debug_configs_names = ['Debug']
            release_configs_names = ['Release']
            for name, config in configs.iteritems():
                if config.derived_from(debug_config):
                    debug_configs_names.append(name)
                elif config.derived_from(release_config):
                    release_configs_names.append(name)

            # Assume that tilde characters are never used in the configuration
            # names (it's certainly not common at the very least).
            non_config_sep = '~~'

            make_test_fmt = 'ifneq (,$(findstring %s$(config)%s,%s%%s%s))' % \
                (non_config_sep, non_config_sep, non_config_sep, non_config_sep)

            make_debug_test = make_test_fmt % non_config_sep.join(debug_configs_names)
            make_release_test = make_test_fmt % non_config_sep.join(release_configs_names)
        else:
            # If we only have the two predefined configs, use simpler tests.
            make_debug_test = 'ifeq ($(config),Debug)'
            make_release_test = 'ifeq ($(config),Release)'

        file.write("""
# You may also specify config=%s
# or their corresponding lower case variants on make command line to select
# the corresponding default flags values.
""" % '|'.join(configs.keys()))

        # Accept configs in lower case too to be more Unix-ish.
        for name in configs:
            file.write(
"""ifeq ($(config),%s)
override config := %s
endif
""" % (name.lower(), name))

        file.write(make_debug_test)
        file.write(
"""
override CPPFLAGS += -DDEBUG
override CFLAGS += -g -O0
override LDFLAGS += -g
else """
)
        file.write(make_release_test)
        file.write(
"""
override CFLAGS += -O2
else ifneq (,$(config))
$(warning Unknown configuration "$(config)")
endif
""")


    def on_header(self, file, module):
        super(XC8Toolset, self).on_header(file, module)

        self.output_default_flags(file, module.project.configurations)

        if module.project.settings:
            file.write("""#
# Additionally, this makefile is customizable with the following
# settings:
#
""")
            alls = [(s.name, s["help"]) for s in module.project.settings.itervalues()]
            width = max(len(x[0]) for x in alls)
            fmtstr = "#      %%-%ds  %%s\n" % width
            for name, doc in alls:
                file.write(fmtstr % (name, doc if doc else ""))

        file.write("""
CC := %s
AR := libr
""" % (self.default_cc))
        # This placeholder will be replaced either with the definition of the
        # macros, if they turn out to be really needed, or nothing otherwise.
        file.write(GMAKE_IFEXPR_MACROS_PLACEHOLDER)
        self.uses_non_std_bool_macros = False

        # Similarly, this one will be replaced with the definition of the
        # build directory variable if we are building any files in this
        # makefile or nothing if we don't (this does happen in top level
        # makefiles which just dispatch the work to other makefiles, no need
        # to clutter them).
        file.write(GMAKE_BUILDDIR_DEF_PLACEHOLDER)


    def _get_builddir_fragment(self, module):
        # Build the value actually representing the build directory, it is
        # only used here (see GnuExprFormatter.path) and only to initialize
        # the internal _builddir in the fragment below.
        makefile = module["%s.makefile" % self.name]
        rel_dir_comps = makefile.components[:-1]

        # None of the complications is needed in the top level makefile,
        # _builddir is the same as $builddir in it anyhow.
        if rel_dir_comps == []:
            builddir_path = "$(builddir)"
        else:
            # Build the relative path to the top source directory.
            to_top_srcdir = "../"*len(rel_dir_comps)

            # First a hack to ensure we start from the top build directory: we
            # need to do this only if the user-defined builddir is relative at
            # make time, so we check for this by comparing it with its absolute
            # path.
            builddir_path = """\
$(if $(findstring $(abspath $(builddir)),$(builddir)),,%s)\
""" % to_top_srcdir

            # Next the build directory itself, whether relative or absolute.
            builddir_path = builddir_path + "$(builddir)"

            # Finally tackle on the relative path to this directory.
            builddir_path = builddir_path + "/" + "/".join(c.as_py() for c in rel_dir_comps)

        return """
# The directory for the build files, may be overridden on make command line.
builddir = .

ifneq ($(builddir),.)
_builddir := %s/
_builddir_error := $(shell mkdir -p $(_builddir) 2>&1)
$(if $(_builddir_error),$(error Failed to create build directory: $(_builddir_error)))
endif
""" % builddir_path


    def on_phony_targets(self, file, targets):
        file.write(".PHONY: %s\n" % " ".join(targets))

    def on_footer(self, file, module):
        file.replace(GMAKE_IFEXPR_MACROS_PLACEHOLDER,
                     GMAKE_IFEXPR_MACROS if self.uses_non_std_bool_macros
                                         else "")

        file.replace(GMAKE_BUILDDIR_DEF_PLACEHOLDER,
                     self._get_builddir_fragment(module) if self.uses_builddir
                                                         else "")


        file.write("\n"
                   "# Dependencies tracking:\n"
                   "-include $(builddir)*.d\n")


