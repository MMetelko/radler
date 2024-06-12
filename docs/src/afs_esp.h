#include RADL_HEADER

#include <jni.h>

class AFS_esp
{
public:
	AFS_esp();
	void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);
private:
	JavaVM *vm;
	JNIEnv *env;
	JavaVMInitArgs vm_args;
};
