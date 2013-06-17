/* 
	max.xray.jit.boidsrender.c by Wesley Smith 12/14/2005
*/

#include "jit.common.h"
#include "max.jit.mop.h"

// a macro to mark exported symbols in the code without requiring an external file to define them
#ifdef WIN_VERSION
	#ifdef C74_EXPORT	// Max SDK 6.1.1+ defines this
		#define T_EXPORT C74_EXPORT
	#else
		// note that this is the required syntax on windows regardless of whether the compiler is msvc or gcc
		#define T_EXPORT __declspec(dllexport)
	#endif
#else // MAC_VERSION
	// usage of this on Mac w/ Max SDK 6.1.1+ is unknown
	// the mac uses the standard gcc syntax, you should also set the -fvisibility=hidden flag to hide the non-marked symbols
	#define T_EXPORT __attribute__((visibility("default")))
#endif

typedef struct _max_xray_jit_boidsrender 
{
	t_object		ob;
	void			*obex;
} t_max_xray_jit_boidsrender;

t_jit_err xray_jit_boidsrender_init(void); 

void *max_xray_jit_boidsrender_new(t_symbol *s, long argc, t_atom *argv);
void max_xray_jit_boidsrender_free(t_max_xray_jit_boidsrender *x);
void *max_xray_jit_boidsrender_class;
		 	
int T_EXPORT main(void)
{	
	void *p,*q;
	
	xray_jit_boidsrender_init();	
	setup((t_messlist **)&max_xray_jit_boidsrender_class, (method)max_xray_jit_boidsrender_new, (method)max_xray_jit_boidsrender_free, (short)sizeof(t_max_xray_jit_boidsrender), 
		0L, A_GIMME, 0);

	p = max_jit_classex_setup(calcoffset(t_max_xray_jit_boidsrender,obex));
	q = jit_class_findbyname(gensym("xray_jit_boidsrender"));    
    max_jit_classex_mop_wrap(p,q,0); 		
    max_jit_classex_standard_wrap(p,q,0); 	
    addmess((method)max_jit_mop_assist, "assist", A_CANT,0);
}

void max_xray_jit_boidsrender_free(t_max_xray_jit_boidsrender *x)
{
	max_jit_mop_free(x);
	jit_object_free(max_jit_obex_jitob_get(x));
	max_jit_obex_free(x);
}

void *max_xray_jit_boidsrender_new(t_symbol *s, long argc, t_atom *argv)
{
	t_max_xray_jit_boidsrender *x;
	void *o;

	if (x=(t_max_xray_jit_boidsrender *)max_jit_obex_new(max_xray_jit_boidsrender_class,gensym("xray_jit_boidsrender"))) {
		if (o=jit_object_new(gensym("xray_jit_boidsrender"))) {
			max_jit_mop_setup_simple(x,o,argc,argv);			
			max_jit_attr_args(x,argc,argv);
		} else {
			error("jit.alphablend: could not allocate object");
			freeobject((t_object *)x);
		}
	}
	return (x);
}
