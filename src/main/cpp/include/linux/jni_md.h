#ifndef _JAVASOFT_JNI_MD_H_
#define _JAVASOFT_JNI_MD_H_

#define JNIEXPORT __attribute__((visibility("default")))
#define JNIIMPORT
#define JNICALL

typedef int jint;
#ifdef _LP64
typedef long jlong;
#else
typedef long long jlong;
#endif

typedef signed char jbyte;

// Compatibility for Windows jni.h usage on Linux
#ifndef __int64
typedef long long __int64;
#endif

#endif
