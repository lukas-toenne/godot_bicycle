import os


def vcproj_sources(sources):
    for x in sources:
        pieces = x.name.split(".")
        if len(pieces) > 0:
            basename = pieces[0]
            basename = basename.replace("\\\\", "/")
            if os.path.isfile(basename + ".c"):
                yield basename + ".c"
            elif os.path.isfile(basename + ".cpp"):
                yield basename + ".cpp"


def vcproj_includes(sources):
    for x in sources:
        pieces = x.name.split(".")
        if len(pieces) > 0:
            basename = pieces[0]
            basename = basename.replace("\\\\", "/")
            if os.path.isfile(basename + ".h"):
                yield basename + ".h"
            elif os.path.isfile(basename + ".hpp"):
                yield basename + ".hpp"


def find_visual_c_batch_file(env):
    from SCons.Tool.MSCommon.vc import (
        get_default_version,
        get_host_target,
        find_batch_file,
    )

    version = get_default_version(env)
    (host_platform, target_platform, _) = get_host_target(env)
    return find_batch_file(env, version, host_platform, target_platform)[0]


def build_commandline(env, commands, num_jobs):
    batch_file = find_visual_c_batch_file(env)
    if not batch_file:
        print("Could not locate Visual Studio batch file to set up the build environment. Building in Visual Studio will not work.")
        return ""

    common_build_prefix = [
        'cmd /V /C set "platform=$(PlatformTarget)"',
        '(if "$(PlatformTarget)"=="x64" (set "platform=x86_amd64"))',
        'call "' + batch_file + '" !plat!',
    ]

    # Windows allows us to have spaces in paths, so we need
    # to double quote off the directory. However, the path ends
    # in a backslash, so we need to remove this, lest it escape the
    # last double quote off, confusing MSBuild
    common_build_postfix = [
        "--directory=\"$(ProjectDir.TrimEnd('\\'))\"",
        "platform=windows",
        "target=$(Configuration)",
        "progress=no",
        "-j%s" % num_jobs,
    ]

    result = " ^& ".join(common_build_prefix + [" ".join([commands] + common_build_postfix)])
    return result


def make_project(env, name, sources, includes):
    if env['vsproj']:
        vs_sources = list(vcproj_sources(sources))
        vs_includes = list(vcproj_includes(includes))

        # This version information (Win32, x64, Debug, Release, Release_Debug seems to be
        # required for Visual Studio to understand that it needs to generate an NMAKE
        # project. Do not modify without knowing what you are doing.
        PLATFORMS = ["Win32", "x64"]
        PLATFORM_IDS = ["32", "64"]
        CONFIGURATIONS = ["Debug", "Release"]
        CONFIGURATION_IDS = ["debug", "release"]
        variants = [
            f'{config}|{platform}'
            for config in CONFIGURATIONS
            for platform in PLATFORMS
        ]
        buildtargets = [
            env['target_path'] + env['target_name'] + f'.{config_id}.{plat_id}'
            for config_id in CONFIGURATION_IDS
            for plat_id in PLATFORM_IDS
        ]

        # buildargs = f" platform={env['platform']}"
        num_jobs = 4
        env["MSVSBUILDCOM"] = build_commandline(env, "scons", num_jobs)
        env["MSVSREBUILDCOM"] = build_commandline(env, "scons vsproj=yes", num_jobs)
        env["MSVSCLEANCOM"] = build_commandline(env, "scons --clean", num_jobs)
        if not env.get("MSVS"):
            env["MSVS"]["PROJECTSUFFIX"] = ".vcxproj"
            env["MSVS"]["SOLUTIONSUFFIX"] = ".sln"
        # env['PDB'] = 
        return env.MSVSProject(target = name + env['MSVSPROJECTSUFFIX'],
                               srcs = vs_sources,
                               incs = vs_includes,
                               buildtarget = buildtargets,
                               variant = variants,
                               auto_build_solution = 0)

    else:
        buildtarget = env['target_path'] + env['platform_path'] + env['target_name']
        return env.SharedLibrary(target=buildtarget, source=sources)
