/*
	max.jit.boids3d.c  12/15/2005 wesley smith

	adapted from:
	boids3d 08/2005 a.sier / jasch adapted from boids by eric singer © 1995-2003 eric l. singer
	free for non-commercial use
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

typedef struct _max_jit_boids2d 
{
	t_object		ob;
	void			*obex;
} t_max_jit_boids2d;

t_jit_err jit_boids2d_init(void); 

void *max_jit_boids2d_new(t_symbol *s, long argc, t_atom *argv);
void max_jit_boids2d_free(t_max_jit_boids2d *x);
void max_jit_boids2d_outputmatrix(t_max_jit_boids2d *x);
void *max_jit_boids2d_class;
		 	
int T_EXPORT main(void)
{	
	void *p,*q;
	
	jit_boids2d_init();	
	setup((t_messlist **)&max_jit_boids2d_class, (method)max_jit_boids2d_new, (method)max_jit_boids2d_free, (short)sizeof(t_max_jit_boids2d), 
		0L, A_GIMME, 0);

	p = max_jit_classex_setup(calcoffset(t_max_jit_boids2d,obex));
	q = jit_class_findbyname(gensym("jit_boids2d"));    
    max_jit_classex_mop_wrap(p,q,MAX_JIT_MOP_FLAGS_OWN_OUTPUTMATRIX|MAX_JIT_MOP_FLAGS_OWN_JIT_MATRIX); 		
    max_jit_classex_standard_wrap(p,q,0); 	
	max_addmethod_usurp_low((method)max_jit_boids2d_outputmatrix, "outputmatrix");	
    addmess((method)max_jit_mop_assist, "assist", A_CANT,0);
}

void max_jit_boids2d_outputmatrix(t_max_jit_boids2d *x)
{
	t_atom a;
	long outputmode=max_jit_mop_getoutputmode(x);
	void *mop=max_jit_obex_adornment_get(x,_jit_sym_jit_mop);
	t_jit_err err;	
	
	if (outputmode&&mop) { //always output unless output mode is none
		if (outputmode==1) {
			if (err=(t_jit_err)jit_object_method(
				max_jit_obex_jitob_get(x), 
				_jit_sym_matrix_calc,
				jit_object_method(mop,_jit_sym_getinputlist),
				jit_object_method(mop,_jit_sym_getoutputlist)))						
			{
				jit_error_code(x,err); 
			} else {
				max_jit_mop_outputmatrix(x);
			}
		}
	}	
}

void max_jit_boids2d_free(t_max_jit_boids2d *x)
{
	max_jit_mop_free(x);
	jit_object_free(max_jit_obex_jitob_get(x));
	max_jit_obex_free(x);
}

void *max_jit_boids2d_new(t_symbol *s, long argc, t_atom *argv)
{
	t_max_jit_boids2d *x;
	void *o;

	if (x=(t_max_jit_boids2d *)max_jit_obex_new(max_jit_boids2d_class,gensym("jit_boids2d"))) {
		if (o=jit_object_new(gensym("jit_boids2d"))) {
			max_jit_mop_setup_simple(x,o,argc,argv);			
			max_jit_attr_args(x,argc,argv);
		} else {
			error("jit.boids3d: could not allocate object");
			freeobject((t_object *)x);
		}
	}
	return (x);
}