
/* This is only a simple script-launcher utility that calls Python interpreter
   on win32 systems, so that running the tools is simpler.
 */

#include <string.h>
#include <stdio.h>
#include <Python.h>
#include <windows.h>

int main(int argc, char** argv)
{
    int ret;
    char filename[2048];
    FILE *fp;
    char *lastp = NULL;

    strncpy(filename, argv[0], 2048);
    if (GetModuleFileName(NULL, (char*)filename, 2048) != 0)
    {
        lastp = strrchr(filename, '.');
    }
    if (lastp == NULL)
    {
        fprintf(stderr, "Incorrect executable name!\n");
        return 1;
    }

    strncpy(lastp, ".py", 2048-(2+lastp-filename));

    fp = fopen(filename, "rb");
    if (fp == NULL)
    {
        fprintf(stderr, "Cannot open script file '%s'!\n", filename);
        return 2;
    }

    Py_Initialize();
    argv[0] = (char*)filename;
    PySys_SetArgv(argc, argv);
    ret = PyRun_SimpleFile(fp, filename);
    Py_Finalize();
    fclose(fp);

    if (ret != 0)
        fprintf(stderr, "Error executing the script!\n");
    return ret;
}
