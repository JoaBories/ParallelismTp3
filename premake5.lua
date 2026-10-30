-- Premake file for KiliEngine2

outputdir = "%{cfg.buildcfg}-%{cfg.system}-%{cfg.architecture}"


workspace "parallelismTp3"
    architecture "x64"

    configurations {
        "Debug",
        "Release"
    }

    debugdir "%{wks.location}"

project ("00_sequentiel")
    location "%{prj.name}"
    kind "ConsoleApp"
    language "C++"

    targetdir ("bin/" .. outputdir .. "/%{prj.name}")
    objdir ("bin-int/" .. outputdir .. "/%{prj.name}")

    files {
        "%{prj.name}/src/**.hpp",
        "%{prj.name}/src/**.cpp"
    }

    includedirs {
        "vendor/glew/include",
        "vendor/glfw/include",
        "vendor/glm/src"
    }

    libdirs {
        "vendor/glew/lib",
        "vendor/glfw/lib"
    }

    links {
        "glew32",
        "glfw3",
        "opengl32"
    }

    postbuildcommands { 
        "{COPYFILE} %[vendor/glew/lib/glew32.dll] %[%{!cfg.targetdir}]",
        "{COPYFILE} %[vendor/glfw/lin/glfw3.dll] %[%{!cfg.targetdir}]",
        "{COPYDIR} %[shaders] %[%{!cfg.targetdir}/shaders]",
    }

    cppdialect "c++20"
    staticruntime "Off"
    systemversion "latest"

    filter "configurations:Debug"
        symbols "On"

    filter "configurations:Release"
        optimize "On"

project ("01_interactions")
    location "%{prj.name}"
    kind "ConsoleApp"
    language "C++"

    targetdir ("bin/" .. outputdir .. "/%{prj.name}")
    objdir ("bin-int/" .. outputdir .. "/%{prj.name}")

    files {
        "%{prj.name}/src/**.hpp",
        "%{prj.name}/src/**.cpp"
    }
    
    includedirs {
        "vendor/glew/include",
        "vendor/glfw/include",
        "vendor/glm/src"
    }
    
    libdirs {
        "vendor/glew/lib",
        "vendor/glfw/lib"
    }
    
    links {
        "glew32",
        "glfw3",
        "opengl32"
    }
    
    postbuildcommands {
        "{COPYFILE} %[vendor/glew/lib/glew32.dll] %[%{!cfg.targetdir}]",
        "{COPYFILE} %[vendor/glfw/lin/glfw3.dll] %[%{!cfg.targetdir}]",
        "{COPYDIR} %[shaders] %[%{!cfg.targetdir}/shaders]",
    }
    
    cppdialect "c++20"
    staticruntime "Off"
    systemversion "latest"
    
    filter "configurations:Debug"
    symbols "On"
    
    filter "configurations:Release"
    optimize "On"

project ("02_affectation")
    location "%{prj.name}"
    kind "ConsoleApp"
    language "C++"
    
    targetdir ("bin/" .. outputdir .. "/%{prj.name}")
    objdir ("bin-int/" .. outputdir .. "/%{prj.name}")
    
    files {
        "%{prj.name}/src/**.hpp",
        "%{prj.name}/src/**.cpp"
    }
    
    includedirs {
        "vendor/glew/include",
        "vendor/glfw/include",
        "vendor/glm/src"
    }
    
    libdirs {
        "vendor/glew/lib",
        "vendor/glfw/lib"
    }
    
    links {
        "glew32",
        "glfw3",
        "opengl32"
    }
    
    postbuildcommands {
        "{COPYFILE} %[vendor/glew/lib/glew32.dll] %[%{!cfg.targetdir}]",
        "{COPYFILE} %[vendor/glfw/lin/glfw3.dll] %[%{!cfg.targetdir}]",
        "{COPYDIR} %[shaders] %[%{!cfg.targetdir}/shaders]",
    }
    
    cppdialect "c++20"
    staticruntime "Off"
    systemversion "latest"
    
    filter "configurations:Debug"
    symbols "On"
    
    filter "configurations:Release"
    optimize "On"
    
project ("03_thread_pool")
    location "%{prj.name}"
    kind "ConsoleApp"
    language "C++"
    
    targetdir ("bin/" .. outputdir .. "/%{prj.name}")
    objdir ("bin-int/" .. outputdir .. "/%{prj.name}")
    
    files {
        "%{prj.name}/src/**.hpp",
        "%{prj.name}/src/**.cpp"
    }
    
    includedirs {
        "vendor/glew/include",
        "vendor/glfw/include",
        "vendor/glm/src"
    }
    
    libdirs {
        "vendor/glew/lib",
        "vendor/glfw/lib"
    }
    
    links {
        "glew32",
        "glfw3",
        "opengl32"
    }
    
    postbuildcommands {
        "{COPYFILE} %[vendor/glew/lib/glew32.dll] %[%{!cfg.targetdir}]",
        "{COPYFILE} %[vendor/glfw/lin/glfw3.dll] %[%{!cfg.targetdir}]",
        "{COPYDIR} %[shaders] %[%{!cfg.targetdir}/shaders]",
    }
    
    cppdialect "c++20"
    staticruntime "Off"
    systemversion "latest"
    
    filter "configurations:Debug"
    symbols "On"
    
    filter "configurations:Release"
    optimize "On"