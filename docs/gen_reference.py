#!/usr/bin/env python

#
#  This file is part of Bakefile (http://www.bakefile.org)
#
#  Copyright (C) 2008-2011 Vaclav Slavik
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

"""
This tool generates reference documentation for Bakefile plugins and puts
it in ref/ directory.
"""

import sys, os, os.path
import shutil
import sphinx.util.docstrings

docs_path = os.path.dirname(sys.argv[0])
bkl_path = os.path.normpath(os.path.join(docs_path, "..", "src"))
sys.path = [bkl_path] + sys.path

import bkl
import bkl.api
import bkl.props

OUT_DIR = os.path.join(docs_path, "ref")
shutil.rmtree(OUT_DIR, ignore_errors=True)
os.makedirs(OUT_DIR)


def write_property(prop):
    desc = ""
    if prop.__doc__:
        desc += prop.__doc__
        desc += "\n"

    if prop.readonly:
        desc += "\n*Read-only property*\n"
    else:
        if prop.default:
            desc += "\n*Default:* ``%s``\n" % prop.default
        else:
            desc += "\n*Required property*\n"

    txt = "**%s** (type: %s)\n\n" % (prop.name, prop.type.name)
    txt += "    " + "\n    ".join(desc.split("\n"))
    txt += "\n"
    return txt


def write_properties(props):
    if not props:
        return ""

    txt = "\n\nProperties\n----------\n\n"

    for p in props:
        txt += write_property(p)

    return txt



DOC_TEMPLATE = """
.. This file was generated by gen_reference.py, don't edit manually!

%(title)s
%(underline)s

%(docstring)s
"""

def write_docs(filename, title, docstring):
    """
    Writes out documentation to a file.
    """

    underline = "".join("=" for i in xrange(0, len(title)))

    f = open("%s/%s.rst" % (OUT_DIR, filename), "wt")
    f.write(DOC_TEMPLATE % locals())


def write_extension_docs(kind, extension, extra_props=[]):
    """
    Writes out documentation for extension of given kind.
    """
    name = extension.name
    print "documenting %s %s..." % (kind, name)
    docstring = sphinx.util.docstrings.prepare_docstring(extension.__doc__)

    title = name
    if len(docstring) > 0:
        summary = docstring[0]
        if summary[-1] == ".":
            title = "%s -- `%s`" % (summary[:-1], name)

    docstring_text = "\n".join(docstring)
    docstring_text += write_properties(list(extension.all_properties()) + extra_props)

    write_docs("%s_%s" % (kind, name), title, docstring_text)




# write docs for all targets:
for t in bkl.api.TargetType.all():
    write_extension_docs("target", t, bkl.props.std_target_props())

# write docs for all toolsets:
for t in bkl.api.Toolset.all():
    write_extension_docs("toolset", t)

# write docs for projects/modules:
write_docs("project", "Global project properties", write_properties(bkl.props.std_project_props()))
write_docs("module", "Module properties", write_properties(bkl.props.std_module_props()))