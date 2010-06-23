/*
 *  Copyright
 */

#ifndef DG_TUTORIAL_API_HH
#define DG_TUTORIAL_API_HH

#if defined (WIN32)
#  ifdef DG_TUTORIAL_EXPORTS
#    define DG_TUTORIAL_EXPORT __declspec(dllexport)
#  else
#    define DG_TUTORIAL_EXPORT __declspec(dllimport)
#  endif
#else
#  define DG_TUTORIAL_EXPORT
#endif

#endif
