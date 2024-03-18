all: mujoco

# VERSION = 3.1.1
# MUJOCO = mujoco-$(VERSION)-linux-x86_64.tar.gz
# TARBALL = build/$(MUJOCO)
# TARBALL_URL = https://github.com/deepmind/mujoco/releases/download/$(VERSION)/$(MUJOCO)
# SOURCE_DIR = build/mujoco-$(VERSION)
# SHA256SUM = b1645edca072d979aa871ebf20313f6a1361153ccb637c0330785c574806520d
# SHA256SUM_ACTUAL = sha256sum $(TARBALL) | cut -d ' ' -f 1
# UNPACK_CMD = tar xzf
# include $(shell rospack find mk)/download_unpack_build.mk

# mujoco: $(SOURCE_DIR)/unpacked
# 	echo "$(SHA256SUM) build/$(MUJOCO)" | sha256sum --check

# clean:
# 	-rm -rf $(SOURCE_DIR)

# wipe: clean
# 	-rm -rf build

VERSION = 2.3.7
MUJOCO = mujoco-$(VERSION)-linux-x86_64.tar.gz
TARBALL = build/$(MUJOCO)
TARBALL_URL = https://github.com/deepmind/mujoco/releases/download/$(VERSION)/$(MUJOCO)
SOURCE_DIR = build/mujoco-$(VERSION)
SHA256SUM = 3f75e53e3356ce4ee6759cde5358b7a53f51632f495b910aedc9caa5bf98ac39
SHA256SUM_ACTUAL = sha256sum $(TARBALL) | cut -d ' ' -f 1
UNPACK_CMD = tar xzf
include $(shell rospack find mk)/download_unpack_build.mk

mujoco: $(SOURCE_DIR)/unpacked
	echo "$(SHA256SUM) build/$(MUJOCO)" | sha256sum --check

clean:
	-rm -rf $(SOURCE_DIR)

wipe: clean
	-rm -rf build