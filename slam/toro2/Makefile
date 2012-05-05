all: installed

SVN_DIR = toro_svn
SVN_URL = https://svn.openslam.org/data/svn/toro
SVN_REVISION = -r 23
SVN_PATCH = fPIC.patch
include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR) patched
	cd $(SVN_DIR)/trunk && make 
	touch installed

clean:
	-cd $(SVN_DIR) && make clean
	rm -rf toro installed

wipe: clean
	rm -rf $(SVN_DIR)
