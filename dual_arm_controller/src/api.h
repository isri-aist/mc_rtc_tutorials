#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define DualArmController_DLLIMPORT __declspec(dllimport)
#  define DualArmController_DLLEXPORT __declspec(dllexport)
#  define DualArmController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define DualArmController_DLLIMPORT __attribute__((visibility("default")))
#    define DualArmController_DLLEXPORT __attribute__((visibility("default")))
#    define DualArmController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define DualArmController_DLLIMPORT
#    define DualArmController_DLLEXPORT
#    define DualArmController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef DualArmController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define DualArmController_DLLAPI
#  define DualArmController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef DualArmController_EXPORTS
#    define DualArmController_DLLAPI DualArmController_DLLEXPORT
#  else
#    define DualArmController_DLLAPI DualArmController_DLLIMPORT
#  endif // DualArmController_EXPORTS
#  define DualArmController_LOCAL DualArmController_DLLLOCAL
#endif // DualArmController_STATIC