#
# Makefile for creating distribution archives.
#

VERSION      := $(shell python ../src/tool.py --version | cut -f2 -d' ')
TARBALL_NAME := bakefile-$(VERSION)_beta


all:


distrib_unix: build
	rm -rf $(TARBALL_NAME)
	mkdir $(TARBALL_NAME)
	(cd .. ; git archive --format=tar HEAD) | (cd $(TARBALL_NAME) ; tar x)
	(cd ../3rdparty/antlr3/python-runtime ; git archive --format=tar HEAD antlr3) | (cd $(TARBALL_NAME)/src ; tar x)
	cp -a ../src/bkl/parser/Bakefile*.py $(TARBALL_NAME)/src/bkl/parser
	cp -a ../docs/html $(TARBALL_NAME)/docs/
	cat ../src/bkl/version.py | sed -e 's/VERSION=".*"/VERSION="$(VERSION)"/' >$(TARBALL_NAME)/src/bkl/version.py
	tar cjf $(TARBALL_NAME)-bin.tar.bz2 $(TARBALL_NAME)
	rm -rf $(TARBALL_NAME)


distrib_win: build pyinstaller/bkl.exe
	rm -rf $(TARBALL_NAME)-win
	mkdir $(TARBALL_NAME)-win
	cp -a pyinstaller/bkl.exe ../COPYING ../README $(TARBALL_NAME)-win
	cp -a ../docs/html $(TARBALL_NAME)-win/docs
	zip -q -r -9 $(TARBALL_NAME)-win.zip $(TARBALL_NAME)-win
	rm -rf $(TARBALL_NAME)-win

pyinstaller/bkl.exe: build
	cp -a ../src/bkl/version.py ../src/bkl/version.py.orig
	cat ../src/bkl/version.py.orig | sed -e 's/VERSION=".*"/VERSION="$(VERSION)"/' >../src/bkl/version.py
	
	rm -rf pyinstaller/_extras
	mkdir -p pyinstaller/_extras/antlr3
	cp -a ../3rdparty/antlr3/python-runtime/antlr3/*.py pyinstaller/_extras/antlr3
	cd pyinstaller && python ../../3rdparty/pyinstaller/pyinstaller.py -p _extras bkl.spec
	rm -rf pyinstaller/_extras
	
	mv -f ../src/bkl/version.py.orig ../src/bkl/version.py

distrib_msi: build pyinstaller/bkl.exe
	cd wix && candle bkl.wxs
	cd wix && light -ext WixUIExtension bkl.wixobj

build:
	make -C ..
