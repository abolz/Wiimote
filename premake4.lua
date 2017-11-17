----------------------------------------------------------------------------------------------------
if _ACTION == "clean" then
    os.rmdir("build")
end

----------------------------------------------------------------------------------------------------
solution "Wiimote"

    configurations { "Release", "Debug" }

    platforms { "x64" }

    location    ("build/" .. _ACTION)
    targetdir   ("build/" .. _ACTION .. "/bin")
    objdir      ("build/" .. _ACTION .. "/obj")

    configuration { "Debug" }
        targetsuffix "d"
        defines { "_DEBUG" }
        flags { "ExtraWarnings", "Symbols" }

    configuration { "Release" }
        defines { "NDEBUG" }
        flags { "ExtraWarnings", "Symbols", "Optimize" }

    configuration { "gmake" }
        buildoptions {
            "-std=c++11",
            "-pedantic",
        }

    configuration { "windows" }
        flags { "Unicode" }

----------------------------------------------------------------------------------------------------
project "Wiimote"

    kind "SharedLib"

    language "C++"

    defines {
        "WIIMOTE_EXPORTS=1",
    }

    includedirs {
        "include/",
    }

    files {
        "include/**",
        "src/**",
    }

    configuration { "windows" }
        links { "winmm", "hid", "setupapi" }

----------------------------------------------------------------------------------------------------
project "Test"

    kind "ConsoleApp"

    language "C++"

    includedirs {
        "include/",
        "test/", -- math library
    }

    files {
        "test/**",
    }

    links { "Wiimote" }

    configuration { "Debug" }
        links { "sfml-system-d", "sfml-window-d", "sfml-graphics-d" }

    configuration { "Release" }
        links { "sfml-system", "sfml-window", "sfml-graphics" }

    configuration { "windows" }
        links { "gdi32", "kernel32", "opengl32", "user32", "winmm" }

    configuration { "vs*" }
        includedirs {
            "$(LIB_PATH)/SFML/include",
        }
        libdirs {
            "$(LIB_PATH)/SFML/lib",
        }

    configuration { "not vs*" }
        links { "boost_thread", "boost_system" }
