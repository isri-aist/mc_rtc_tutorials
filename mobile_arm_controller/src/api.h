#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define MobileArmController_DLLIMPORT __declspec(dllimport)
#  define MobileArmController_DLLEXPORT __declspec(dllexport)
#  define MobileArmController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define MobileArmController_DLLIMPORT __attribute__((visibility("default")))
#    define MobileArmController_DLLEXPORT __attribute__((visibility("default")))
#    define MobileArmController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define MobileArmController_DLLIMPORT
#    define MobileArmController_DLLEXPORT
#    define MobileArmController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef MobileArmController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define MobileArmController_DLLAPI
#  define MobileArmController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef MobileArmController_EXPORTS
#    define MobileArmController_DLLAPI MobileArmController_DLLEXPORT
#  else
#    define MobileArmController_DLLAPI MobileArmController_DLLIMPORT
#  endif // MobileArmController_EXPORTS
#  define MobileArmController_LOCAL MobileArmController_DLLLOCAL
#endif // MobileArmController_STATIC