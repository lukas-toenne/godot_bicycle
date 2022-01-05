#!python
import subprocess

godot_cpp_dir = "godot-cpp"
platform = "windows"
archs = ["64", "32"]
targets = ["debug", "release"]

for bits in archs:
    for target in targets:
        process = subprocess.Popen(
            ["scons", f"platform={platform}", "generate_bindings=yes", "-j4", f"bits={bits}", f"target={target}"],
            cwd=godot_cpp_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            bufsize=0)

        print(process.args)
        for line in process.stdout:
            print(line.strip())
