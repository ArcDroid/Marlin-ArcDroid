from os.path import join, isfile
import os
import distutils.spawn

Import("env")

FRAMEWORK_DIR = env.PioPlatform().get_package_dir("framework-arduino-lpc176x")
patchflag_path = join(FRAMEWORK_DIR, ".patching-done")

# patch file only if we didn't do it before
if not isfile(join(FRAMEWORK_DIR, ".patching-done")):
    original_file = join(FRAMEWORK_DIR, "cores", "arduino", "SoftwareSerial.cpp")
    patched_file = join("buildroot", "share", "PlatformIO", "patches", "lpc176x-software-serial-115200.patch")
    print("original_file: {:}".format(original_file))
    print("patched_file : {:}".format(patched_file))
    print("cwd: {:}".format(os.getcwd()))

    assert isfile(original_file) and isfile(patched_file)

    if (distutils.spawn.find_executable("patch") is None):
        os.environ["PATH"] += os.pathsep + "C:\\Program Files\\Git\\usr\\bin"

    print("executing: patch %s %s" % (original_file, patched_file))
    env.Execute("patch %s %s" % (original_file, patched_file))
    # env.Execute("touch " + patchflag_path)


    def _touch(path):
        with open(path, "w") as fp:
            fp.write("")

    env.Execute(lambda *args, **kwargs: _touch(patchflag_path))
