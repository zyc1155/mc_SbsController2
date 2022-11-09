#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define SbsControllerZyc_DLLIMPORT __declspec(dllimport)
#  define SbsControllerZyc_DLLEXPORT __declspec(dllexport)
#  define SbsControllerZyc_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define SbsControllerZyc_DLLIMPORT __attribute__((visibility("default")))
#    define SbsControllerZyc_DLLEXPORT __attribute__((visibility("default")))
#    define SbsControllerZyc_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define SbsControllerZyc_DLLIMPORT
#    define SbsControllerZyc_DLLEXPORT
#    define SbsControllerZyc_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef SbsControllerZyc_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define SbsControllerZyc_DLLAPI
#  define SbsControllerZyc_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef SbsControllerZyc_EXPORTS
#    define SbsControllerZyc_DLLAPI SbsControllerZyc_DLLEXPORT
#  else
#    define SbsControllerZyc_DLLAPI SbsControllerZyc_DLLIMPORT
#  endif // SbsControllerZyc_EXPORTS
#  define SbsControllerZyc_LOCAL SbsControllerZyc_DLLLOCAL
#endif // SbsControllerZyc_STATIC