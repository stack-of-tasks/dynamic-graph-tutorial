/*
 *  Copyright
 */

#ifndef SOT_TUTORIAL_API_HH
#define SOT_TUTORIAL_API_HH

#if defined (WIN32)
#  ifdef SOT_TUTORIAL_EXPORTS
#    define SOT_TUTORIAL_EXPORT __declspec(dllexport)
#  else
#    define SOT_TUTORIAL_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOT_TUTORIAL_EXPORT
#endif

#endif
