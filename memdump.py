Import("env")

#
# Dump build environment (for debug)
# print env.Dump()
#

#
# Upload actions
#

def before_upload(source, target, env):
    print("**** before upload")

def after_build(source, target, env):
    print("****SOURCE: " + str(map(str, source)))
    print("****TARGET: " + str(map(str, target)))

def after_upload(source, target, env):
    print "after_upload"
    # do some actions

print "****** Current build targets", map(str, BUILD_TARGETS)

env.AddPreAction("upload", before_upload)
env.AddPostAction("upload", after_upload)
#env.AddPreAction("", before_build)
env.AddPostAction("buildprog", after_build)


dict = env.Dictionary('PIOPLATFORM')
print("****TARGET: " + dict)
if dict == "atmelsam":
    env.AddPostAction("$BUILD_DIR/firmware.elf", env.VerboseAction(" ".join(["arm-none-eabi-nm", "-Cr", "-td", "--size-sort", "$BUILD_DIR/firmware.elf"]), "Building $BUILD_DIR/firmware.hex"))
else:
    env.AddPostAction("$BUILD_DIR/firmware.elf", env.VerboseAction(" ".join(["avr-nm", "-Crtd", "--size-sort", "$BUILD_DIR/firmware.elf", "|", "grep", "-i", "' [dbv] '"]), "Building $BUILD_DIR/firmware.hex"))
