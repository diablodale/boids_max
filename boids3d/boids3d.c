/*

	boids3d 08/2005 a.sier / jasch adapted from boids by eric singer © 1995-2003 eric l. singer
	free for non-commercial use
*/

#include	<ext.h>
#include	<ext_common.h>
#include	<stdlib.h>
#include	<math.h>

// constants
#define			kAssistInlet	1
#define			kAssistOutlet	2
#define			kMaxLong		0xFFFFFFFF
#define			kMaxNeighbors	4

// initial flight parameters
const short			kNumBoids		= 12;	// number of boids
const short			kNumNeighbors	= 2;	// must be <= kMaxNeighbors
const double 		kMinSpeed		= 0.15;	// boids' minimum speed
const double		kMaxSpeed		= 0.25;	// boids' maximum speed
const double		kCenterWeight	= 0.25;	// flock centering
const double		kAttractWeight	= 0.300;// attraction point seeking
const double		kMatchWeight	= 0.100;// neighbors velocity matching
const double		kAvoidWeight	= 0.10;	// neighbors avoidance
const double		kWallsWeight	= 0.500;// wall avoidance [210]
const double		kEdgeDist		= 0.5;	// vision distance to avoid wall edges [5]
const double		kSpeedupFactor	= 0.100;// alter animation speed
const double		kInertiaFactor	= 0.20;	// willingness to change speed & direction
const double		kAccelFactor	= 0.100;// neighbor avoidance accelerate or decelerate rate
const double		kPrefDist		= 0.25;	// preferred distance from neighbors
const double		kFlyRectTop		= 1.0;	// fly rect boundaries
const double		kFlyRectLeft	= -1.0;	
const double		kFlyRectBottom	= -1.0;	
const double		kFlyRectRight	= 1.0;	
const double		kFlyRectFront	= 1.0;	
const double		kFlyRectBack	= -1.0;	


// typedefs
typedef struct Velocity {
	double		x;
	double		y;
	double		z;
} Velocity;

typedef struct Point3d {
	double		x;
	double		y;
	double		z;
} Point3d;

typedef struct Box3D {
	double		left, right;
	double		top, bottom;
	double		front, back;
} Box3D;

typedef struct Boid {
	Point3d		oldPos;
	Point3d		newPos;
	Velocity	oldDir;
	Velocity	newDir;
	double		speed;
	short		neighbor[kMaxNeighbors];
	double		neighborDistSqr[kMaxNeighbors];
} Boid, *BoidPtr;

typedef struct FlockObject {
	Object		theObject;
	void		*out1, *out2;
	short		mode;
	long		numBoids;
	long		numNeighbors;
	Box3D		flyRect;
	double 		minSpeed;
	double		maxSpeed;
	double		centerWeight;
	double		attractWeight;
	double		matchWeight;
	double		avoidWeight;
	double		wallsWeight;
	double		edgeDist;
	double		speedupFactor;
	double		inertiaFactor;
	double		accelFactor;
	double		prefDist;
	double		prefDistSqr;
	Point3d		centerPt;
	Point3d		attractPt;
	BoidPtr		boid;
	double 		d2r, r2d;
} FlockObject, *FlockPtr;

// variables
void*	flock;
t_symbol *ps_nothing;

// prototypes
void main();
void* Flock_new(long numBoids, long mode);
void Flock_free(FlockPtr flockPtr);
void Flock_assist(FlockPtr flockPtr, void* temp, long letType, long letNum, char *assistStr);
void Flock_bang(FlockPtr flockPtr);
void Flock_dump(FlockPtr flockPtr);
void Flock_mode(FlockPtr flockPtr, long arg);
void Flock_numNeighbors(FlockPtr flockPtr, long arg);
void Flock_numBoids(FlockPtr flockPtr, long arg);
void Flock_donumBoids(FlockPtr flockPtr, Symbol *msg, short argc, Atom *argv);
void Flock_minSpeed(FlockPtr flockPtr, double arg);
void Flock_maxSpeed(FlockPtr flockPtr, double arg);
void Flock_centerWeight(FlockPtr flockPtr, double arg);
void Flock_attractWeight(FlockPtr flockPtr, double arg);
void Flock_matchWeight(FlockPtr flockPtr, double arg);
void Flock_avoidWeight(FlockPtr flockPtr, double arg);
void Flock_wallsWeight(FlockPtr flockPtr, double arg);
void Flock_edgeDist(FlockPtr flockPtr, double arg);
void Flock_speedupFactor(FlockPtr flockPtr, double arg);
void Flock_inertiaFactor(FlockPtr flockPtr, double arg);
void Flock_accelFactor(FlockPtr flockPtr, double arg);
void Flock_prefDist(FlockPtr flockPtr, double arg);
void Flock_flyRect(FlockPtr flockPtr, Symbol *msg, short argc, Atom *argv);
void Flock_attractPt(FlockPtr flockPtr, Symbol *msg, short argc, Atom *argv);
void Flock_reset(FlockPtr flockPtr);
void Flock_resetBoids(FlockPtr flockPtr);
void InitFlock(FlockPtr flockPtr);
void FlightStep(FlockPtr flockPtr);
Point3d FindFlockCenter(FlockPtr flockPtr);
float MatchAndAvoidNeighbors(FlockPtr flockPtr, short theBoid, Velocity *matchNeighborVel, Velocity *avoidNeighborVel);
Velocity SeekPoint(FlockPtr flockPtr, short theBoid, Point3d seekPt);
Velocity AvoidWalls(FlockPtr flockPtr, short theBoid);
Boolean InFront(BoidPtr theBoid, BoidPtr neighbor);
void NormalizeVelocity(Velocity *direction);
double RandomInt(double minRange, double maxRange);
double DistSqrToPt(Point3d firstPoint, Point3d secondPoint);

void main()
{
	setup((t_messlist **) &flock, (method) Flock_new, (method) Flock_free, (short) sizeof(FlockObject), 0L, A_LONG, A_DEFLONG, 0);
	addmess((method) Flock_assist,"assist",	A_CANT, 0);
	addint((method)  Flock_numBoids);
	addbang((method) Flock_bang);
	addmess((method) Flock_numNeighbors,	"neighbors", 	A_LONG, 0);
	addmess((method) Flock_numBoids,		"number", 		A_LONG, 0);
	addmess((method) Flock_mode,			"mode", 		A_LONG, 0);
	addmess((method) Flock_minSpeed,		"minspeed", 	A_FLOAT, 0);
	addmess((method) Flock_maxSpeed,		"maxspeed", 	A_FLOAT, 0);
	addmess((method) Flock_centerWeight,	"center", 		A_FLOAT, 0);
	addmess((method) Flock_attractWeight,	"attract", 		A_FLOAT, 0);
	addmess((method) Flock_matchWeight,		"match", 		A_FLOAT, 0);
	addmess((method) Flock_avoidWeight,		"avoid", 		A_FLOAT, 0);
	addmess((method) Flock_wallsWeight,		"repel", 		A_FLOAT, 0);
	addmess((method) Flock_edgeDist,		"edgedist", 	A_FLOAT, 0);
	addmess((method) Flock_speedupFactor,	"speed", 		A_FLOAT, 0);
	addmess((method) Flock_inertiaFactor,	"inertia",	 	A_FLOAT, 0);
	addmess((method) Flock_accelFactor,		"accel", 		A_FLOAT, 0);
	addmess((method) Flock_prefDist,		"prefdist", 	A_FLOAT, 0);
	addmess((method) Flock_flyRect,			"flyrect", 		A_GIMME, 0);
	addmess((method) Flock_attractPt, 		"attractpt", 	A_GIMME, 0);
	addmess((method) Flock_resetBoids, 		"reset", 		0);
	addmess((method) Flock_reset, 			"init", 		0);
	addmess((method) Flock_dump, 			"dump", 		0);
	
	post("boids3d 2005 a.sier / jasch   © 1995-2003 eric l. singer   "__DATE__" "__TIME__);	
	ps_nothing = gensym("");
}


void* Flock_new(long numBoids, long mode)
{
	FlockPtr	flockPtr;
	short		tempmode;
	t_atom 		temp[1];
	t_symbol 	*s = ps_nothing;
	
	flockPtr = (FlockPtr) newobject(flock);
	flockPtr->out2 = outlet_new(flockPtr, 0L);
	flockPtr->out1 = outlet_new(flockPtr, 0L);
	
	flockPtr->boid = (Boid *) sysmem_newptr(sizeof(Boid));
	
	temp[0].a_type = A_LONG;
	temp[0].a_w.w_long =  numBoids;
	defer(flockPtr, (method)Flock_donumBoids, s, 1, temp);

	flockPtr->numBoids = numBoids;

	InitFlock(flockPtr);
	
	if(mode){
 		flockPtr->mode = CLIP(mode, 0, 2);
	}else{
		flockPtr->mode = 0;
	}	
	flockPtr->d2r = 3.141592653589793238462643383279502884197169399375105820974944592307816406286208998628034825342117068/180.0;
	flockPtr->r2d = 180.0/3.141592653589793238462643383279502884197169399375105820974944592307816406286208998628034825342117068;
	return(flockPtr);
}


void Flock_free(FlockPtr flockPtr)
{
	sysmem_freeptr((Ptr)flockPtr->boid);
}


void Flock_assist(FlockPtr flockPtr, void* temp, long letType, long letNum, char *assistStr)
{
	switch (letType){
		case kAssistInlet:
			switch (letNum) {
				case 0:
					sprintf(assistStr, "bangs & parameter messages");
					break;
			}
			break;
		case kAssistOutlet:
			switch (letNum) {
				case 0:
					sprintf(assistStr, "boids data (xyz...)");
				break;
				case 1:	
					sprintf(assistStr, "parameter dump");
				break;
			}
			break;
	}
}

void Flock_bang(FlockPtr flockPtr)
{
	short	i;
	t_atom 	outlist[10];
	t_atom 	*out;
	
	double 	tempNew_x, tempNew_y, tempNew_z;
	double 	tempOld_x, tempOld_y, tempOld_z;
	double	delta_x, delta_y, delta_z, azi, ele, speed;
	// double tempspeed;
	
	out = outlist;
		
	FlightStep(flockPtr);


	switch(flockPtr->mode) { // newpos
		case 0:
		for (i = 0; i < flockPtr->numBoids; i++){
			SETLONG(out+0, i);
			SETFLOAT(out+1, flockPtr->boid[i].newPos.x);
			SETFLOAT(out+2, flockPtr->boid[i].newPos.y);
			SETFLOAT(out+3, flockPtr->boid[i].newPos.z);
			outlet_list(flockPtr->out1, 0L, 4, out);
		}
		break;
		case 1: //newpos + oldpos
		for (i = 0; i < flockPtr->numBoids; i++){
			SETLONG(out+0, i);
			SETFLOAT(out+1, flockPtr->boid[i].newPos.x);
			SETFLOAT(out+2, flockPtr->boid[i].newPos.y);
			SETFLOAT(out+3, flockPtr->boid[i].newPos.z);
			SETFLOAT(out+4, flockPtr->boid[i].oldPos.x);
			SETFLOAT(out+5, flockPtr->boid[i].oldPos.y);
			SETFLOAT(out+6, flockPtr->boid[i].oldPos.z);
			outlet_list(flockPtr->out1, 0L, 7, out);
		}
		break;
		case 2:
		for (i = 0; i < flockPtr->numBoids; i++){						
			tempNew_x = flockPtr->boid[i].newPos.x;
			tempNew_y = flockPtr->boid[i].newPos.y;
			tempNew_z = flockPtr->boid[i].newPos.z;
			tempOld_x = flockPtr->boid[i].oldPos.x;
			tempOld_y = flockPtr->boid[i].oldPos.y;
			tempOld_z = flockPtr->boid[i].oldPos.z;
			delta_x = tempNew_x - tempOld_x;
			delta_y = tempNew_y - tempOld_y;
			delta_z = tempNew_z - tempOld_z;
			azi = atan2(delta_z, delta_x) * flockPtr->r2d;
			ele = atan2(delta_y, delta_x) * flockPtr->r2d;
			speed = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
			SETLONG(out+0, i);
			SETFLOAT(out+1, tempNew_x);
			SETFLOAT(out+2, tempNew_y);
			SETFLOAT(out+3, tempNew_z);
			SETFLOAT(out+4, tempOld_x);
			SETFLOAT(out+5, tempOld_y);
			SETFLOAT(out+6, tempOld_z);
			SETFLOAT(out+7, speed);
			SETFLOAT(out+8, azi);
			SETFLOAT(out+9, ele);
			outlet_list(flockPtr->out1, 0L, 10, out);
		}
		break;	
	}
}

void Flock_dump(FlockPtr flockPtr)
{
	Atom	outList[6];
	
	outList[0].a_type = A_LONG;
	outList[0].a_w.w_long =  flockPtr->numNeighbors;
	outlet_anything(flockPtr->out2, gensym("neighbors"), 1, outList);

	outList[0].a_type = A_FLOAT;
	outList[0].a_w.w_float =  flockPtr->minSpeed;
	outlet_anything(flockPtr->out2, gensym("minspeed"), 1, outList);

	outList[0].a_type = A_FLOAT;
	outList[0].a_w.w_float =  flockPtr->maxSpeed;
	outlet_anything(flockPtr->out2, gensym("maxspeed"), 1, outList);

	outList[0].a_type = A_FLOAT;
	outList[0].a_w.w_float =  flockPtr->centerWeight;
	outlet_anything(flockPtr->out2, gensym("center"), 1, outList);

	outList[0].a_type = A_FLOAT;
	outList[0].a_w.w_float =  flockPtr->attractWeight;
	outlet_anything(flockPtr->out2, gensym("attract"), 1, outList);

	outList[0].a_type = A_FLOAT;
	outList[0].a_w.w_float =  flockPtr->matchWeight;
	outlet_anything(flockPtr->out2, gensym("match"), 1, outList);

	outList[0].a_type = A_FLOAT;
	outList[0].a_w.w_float =  flockPtr->avoidWeight;
	outlet_anything(flockPtr->out2, gensym("avoid"), 1, outList);

	outList[0].a_type = A_FLOAT;
	outList[0].a_w.w_float =  flockPtr->wallsWeight;
	outlet_anything(flockPtr->out2, gensym("repel"), 1, outList);

	outList[0].a_type = A_FLOAT;
	outList[0].a_w.w_float =  flockPtr->edgeDist;
	outlet_anything(flockPtr->out2, gensym("edgedist"), 1, outList);

	outList[0].a_type = A_FLOAT;
	outList[0].a_w.w_float =  flockPtr->speedupFactor;
	outlet_anything(flockPtr->out2, gensym("speed"), 1, outList);

	outList[0].a_type = A_FLOAT;
	outList[0].a_w.w_float =  flockPtr->inertiaFactor;
	outlet_anything(flockPtr->out2, gensym("inertia"), 1, outList);

	outList[0].a_type = A_FLOAT;
	outList[0].a_w.w_float =  flockPtr->accelFactor;
	outlet_anything(flockPtr->out2, gensym("accel"), 1, outList);

	outList[0].a_type = A_FLOAT;
	outList[0].a_w.w_float = flockPtr->prefDist;
	outlet_anything(flockPtr->out2, gensym("prefdist"), 1, outList);

	outList[0].a_type = A_FLOAT;
	outList[0].a_w.w_float =  flockPtr->flyRect.left;
	outList[1].a_type = A_FLOAT;
	outList[1].a_w.w_float =  flockPtr->flyRect.top;
	outList[2].a_type = A_FLOAT;
	outList[2].a_w.w_float =  flockPtr->flyRect.right;
	outList[3].a_type = A_FLOAT;
	outList[3].a_w.w_float =  flockPtr->flyRect.bottom;
	outList[4].a_type = A_FLOAT;
	outList[4].a_w.w_float =  flockPtr->flyRect.front;
	outList[5].a_type = A_FLOAT;
	outList[5].a_w.w_float =  flockPtr->flyRect.back;
	outlet_anything(flockPtr->out2, gensym("flyrect"), 6, outList);

	outList[0].a_type = A_FLOAT;
	outList[0].a_w.w_float =  flockPtr->attractPt.x;
	outList[1].a_type = A_FLOAT;
	outList[1].a_w.w_float =  flockPtr->attractPt.y;
	outList[2].a_type = A_FLOAT;
	outList[2].a_w.w_float =  flockPtr->attractPt.z;
	outlet_anything(flockPtr->out2, gensym("attractpt"), 3, outList);
	
	outList[0].a_type = A_LONG;
	outList[0].a_w.w_long =  flockPtr->mode;
	outlet_anything(flockPtr->out2, gensym("mode"), 1, outList);
	
	outList[0].a_type = A_LONG;
	outList[0].a_w.w_long =  flockPtr->numBoids;
	outlet_anything(flockPtr->out2, gensym("number"), 1, outList);
	
}

void Flock_mode(FlockPtr flockPtr, long arg)
{
	flockPtr->mode = CLIP(arg, 0, 2);
}

void Flock_numNeighbors(FlockPtr flockPtr, long arg)
{
	flockPtr->numNeighbors = arg;
}

void Flock_numBoids(FlockPtr flockPtr, long arg)
{
	t_atom temp[1];
	t_symbol *s = ps_nothing;
	
	temp[0].a_type = A_LONG;
	temp[0].a_w.w_long =  arg;
	defer(flockPtr, (method)Flock_donumBoids, s, 1, temp);
	Flock_resetBoids(flockPtr);
}

void Flock_donumBoids(FlockPtr flockPtr, Symbol *msg, short argc, Atom *argv)
{
		flockPtr->boid = (Boid *)sysmem_resizeptr(flockPtr->boid, sizeof(Boid) * argv[0].a_w.w_long);
		flockPtr->numBoids = argv[0].a_w.w_long;
}

void Flock_minSpeed(FlockPtr flockPtr, double arg)
{
	flockPtr->minSpeed = MAX(arg, 0.000001);
}

void Flock_maxSpeed(FlockPtr flockPtr, double arg)
{
	flockPtr->maxSpeed = arg;
}

void Flock_centerWeight(FlockPtr flockPtr, double arg)
{
	flockPtr->centerWeight = arg;
}

void Flock_attractWeight(FlockPtr flockPtr, double arg)
{
	flockPtr->attractWeight = arg;
}

void Flock_matchWeight(FlockPtr flockPtr, double arg)
{
	flockPtr->matchWeight = arg;
}

void Flock_avoidWeight(FlockPtr flockPtr, double arg)
{
	flockPtr->avoidWeight = arg;
}

void Flock_wallsWeight(FlockPtr flockPtr, double arg)
{
	flockPtr->wallsWeight = arg;
}

void Flock_edgeDist(FlockPtr flockPtr, double arg)
{
	flockPtr->edgeDist = arg;
}

void Flock_speedupFactor(FlockPtr flockPtr, double arg)
{
	flockPtr->speedupFactor = arg;
}

void Flock_inertiaFactor(FlockPtr flockPtr, double arg)
{
	if(arg == 0){
		flockPtr->inertiaFactor = 0.000001;
	}else{
		flockPtr->inertiaFactor = arg;
	}
}

void Flock_accelFactor(FlockPtr flockPtr, double arg)
{
	flockPtr->accelFactor = arg;
}

void Flock_prefDist(FlockPtr flockPtr, double arg)
{
	flockPtr->prefDist = arg;
}

void Flock_flyRect(FlockPtr flockPtr, Symbol *msg, short argc, Atom *argv)
{
	double temp[6];
	short i;
	if(argc == 6){
		for(i=0;i<6;i++) {
			if(argv->a_type == A_FLOAT) {
				temp[i] = (double)argv->a_w.w_float;	
				argv++;
			} else if(argv->a_type == A_LONG) {
				temp[i] = (double)argv->a_w.w_long;	
				argv++;
			}
		}
		flockPtr->flyRect.left 		= temp[0];
		flockPtr->flyRect.top 		= temp[1];
		flockPtr->flyRect.right 	= temp[2];
		flockPtr->flyRect.bottom 	= temp[3];
		flockPtr->flyRect.front 	= temp[4];
		flockPtr->flyRect.back 		= temp[5];
	}else{
		error("boids3d: flyrect needs six values");
	}
}

void Flock_attractPt(FlockPtr flockPtr, Symbol *msg, short argc, Atom *argv)
{
	double temp[3];
	short i;
	if(argc == 3){
		for(i=0;i<3;i++) {
			if(argv->a_type == A_FLOAT) {
				temp[i] = (double)argv->a_w.w_float;	
				argv++;
			} else if(argv->a_type == A_LONG) {
				temp[i] = (double)argv->a_w.w_long;	
				argv++;
			}
		}
		flockPtr->attractPt.x = temp[0];
		flockPtr->attractPt.y = temp[1];
		flockPtr->attractPt.z = temp[2];
	}else{
		error("boids3d: attractPt needs three values");
	}
}

void Flock_reset(FlockPtr flockPtr)
{
	InitFlock(flockPtr);
}

void Flock_resetBoids(FlockPtr flockPtr)
{
	long i, j;
	double rndAngle;
	
	for (i = 0; i <  flockPtr->numBoids; i++) { // init everything to 0.0
		flockPtr->boid[i].oldPos.x = 0.0;
		flockPtr->boid[i].oldPos.y = 0.0;
		flockPtr->boid[i].oldPos.z = 0.0;

		flockPtr->boid[i].newPos.x = 0.0;
		flockPtr->boid[i].newPos.y = 0.0;
		flockPtr->boid[i].newPos.z = 0.0;
		
		flockPtr->boid[i].oldDir.x = 0.0;
		flockPtr->boid[i].oldDir.y = 0.0;
		flockPtr->boid[i].oldDir.z = 0.0;
		
		flockPtr->boid[i].newDir.x = 0.0;
		flockPtr->boid[i].newDir.y = 0.0;
		flockPtr->boid[i].newDir.z = 0.0;
		
		flockPtr->boid[i].speed = 0.0;
		
		for(j=0; j<kMaxNeighbors;j++){
			flockPtr->boid[i].neighbor[j] = 0;
			flockPtr->boid[i].neighborDistSqr[j] = 0.0;
		}
	}
	for (i = 0; i <  flockPtr->numBoids; i++) {				// set the initial locations and velocities of the boids
		flockPtr->boid[i].newPos.x = flockPtr->boid[i].oldPos.x = RandomInt(flockPtr->flyRect.right,flockPtr->flyRect.left);		// set random location within flyRect
		flockPtr->boid[i].newPos.y = flockPtr->boid[i].oldPos.y = RandomInt(flockPtr->flyRect.bottom, flockPtr->flyRect.top);
		flockPtr->boid[i].newPos.z = flockPtr->boid[i].oldPos.z = RandomInt(flockPtr->flyRect.back, flockPtr->flyRect.front);
		rndAngle = RandomInt(0, 360) * flockPtr->d2r;		// set velocity from random angle
		flockPtr->boid[i].newDir.x = sin(rndAngle);
		flockPtr->boid[i].newDir.y = cos(rndAngle);
		flockPtr->boid[i].newDir.z = (cos(rndAngle) + sin(rndAngle)) * 0.5;
		flockPtr->boid[i].speed = (kMaxSpeed + kMinSpeed) * 0.5;
	}

}

void InitFlock(FlockPtr flockPtr)
{
	flockPtr->numNeighbors		= kNumNeighbors;
	flockPtr->minSpeed			= kMinSpeed;
	flockPtr->maxSpeed			= kMaxSpeed;
	flockPtr->centerWeight		= kCenterWeight;
	flockPtr->attractWeight		= kAttractWeight;
	flockPtr->matchWeight		= kMatchWeight;
	flockPtr->avoidWeight		= kAvoidWeight;
	flockPtr->wallsWeight		= kWallsWeight;
	flockPtr->edgeDist			= kEdgeDist;
	flockPtr->speedupFactor		= kSpeedupFactor;
	flockPtr->inertiaFactor		= kInertiaFactor;
	flockPtr->accelFactor		= kAccelFactor;
	flockPtr->prefDist			= kPrefDist;
	flockPtr->prefDistSqr		= kPrefDist * kPrefDist;
	flockPtr->flyRect.top		= kFlyRectTop;
	flockPtr->flyRect.left		= kFlyRectLeft;
	flockPtr->flyRect.bottom	= kFlyRectBottom;
	flockPtr->flyRect.right		= kFlyRectRight;
	flockPtr->flyRect.front		= kFlyRectFront;
	flockPtr->flyRect.back		= kFlyRectBack;
	flockPtr->attractPt.x		= (kFlyRectLeft + kFlyRectRight) * 0.5;
	flockPtr->attractPt.y		= (kFlyRectTop + kFlyRectBottom) * 0.5;
	flockPtr->attractPt.z		= (kFlyRectFront + kFlyRectBack) * 0.5;
	Flock_resetBoids(flockPtr);
}

void FlightStep(FlockPtr flockPtr)
{
	Velocity		goCenterVel;
	Velocity		goAttractVel;
	Velocity		matchNeighborVel;
	Velocity		avoidWallsVel;
	Velocity		avoidNeighborVel;
	float			avoidNeighborSpeed;
	const Velocity	zeroVel	= {0.0, 0.0, 0.0};
	short			i;

	flockPtr->centerPt = FindFlockCenter(flockPtr);
	for (i = 0; i <  flockPtr->numBoids; i++) {						// save position and velocity
		flockPtr->boid[i].oldPos.x = flockPtr->boid[i].newPos.x;
		flockPtr->boid[i].oldPos.y = flockPtr->boid[i].newPos.y;
		flockPtr->boid[i].oldPos.z = flockPtr->boid[i].newPos.z;
		
		flockPtr->boid[i].oldDir.x = flockPtr->boid[i].newDir.x;
		flockPtr->boid[i].oldDir.y = flockPtr->boid[i].newDir.y;
		flockPtr->boid[i].oldDir.z = flockPtr->boid[i].newDir.z;
	}
	for (i = 0; i < flockPtr->numBoids; i++) {
		if (flockPtr->numNeighbors > 0) {							// get all velocity components
			avoidNeighborSpeed = MatchAndAvoidNeighbors(flockPtr, i,&matchNeighborVel, &avoidNeighborVel);
		} else {
			matchNeighborVel = zeroVel;
			avoidNeighborVel = zeroVel;
			avoidNeighborSpeed = 0;
		}
		goCenterVel = SeekPoint(flockPtr, i, flockPtr->centerPt);			
		goAttractVel = SeekPoint(flockPtr, i, flockPtr->attractPt);
		avoidWallsVel = AvoidWalls(flockPtr, i);
	
		// compute resultant velocity using weights and inertia
		flockPtr->boid[i].newDir.x = flockPtr->inertiaFactor * (flockPtr->boid[i].oldDir.x) +
							(flockPtr->centerWeight * goCenterVel.x +
							 flockPtr->attractWeight * goAttractVel.x +
							 flockPtr->matchWeight * matchNeighborVel.x +
							 flockPtr->avoidWeight * avoidNeighborVel.x +
							 flockPtr->wallsWeight * avoidWallsVel.x) / flockPtr->inertiaFactor;
		flockPtr->boid[i].newDir.y = flockPtr->inertiaFactor * (flockPtr->boid[i].oldDir.y) +
							(flockPtr->centerWeight * goCenterVel.y +
							 flockPtr->attractWeight * goAttractVel.y +
							 flockPtr->matchWeight * matchNeighborVel.y +
							 flockPtr->avoidWeight * avoidNeighborVel.y +
							 flockPtr->wallsWeight * avoidWallsVel.y) / flockPtr->inertiaFactor;
		flockPtr->boid[i].newDir.z = flockPtr->inertiaFactor * (flockPtr->boid[i].oldDir.z) +
							(flockPtr->centerWeight * goCenterVel.z +
							 flockPtr->attractWeight * goAttractVel.z +
							 flockPtr->matchWeight * matchNeighborVel.z +
							 flockPtr->avoidWeight * avoidNeighborVel.z +
							 flockPtr->wallsWeight * avoidWallsVel.z) / flockPtr->inertiaFactor;
		NormalizeVelocity(&(flockPtr->boid[i].newDir));	// normalize velocity so its length is unity

		// set to avoidNeighborSpeed bounded by minSpeed and maxSpeed
		if ((avoidNeighborSpeed >= flockPtr->minSpeed) &&
				(avoidNeighborSpeed <= flockPtr->maxSpeed))
			flockPtr->boid[i].speed = avoidNeighborSpeed;
		else if (avoidNeighborSpeed > flockPtr->maxSpeed)
			flockPtr->boid[i].speed = flockPtr->maxSpeed;
		else
			flockPtr->boid[i].speed = flockPtr->minSpeed;

		// calculate new position, applying speedupFactor
		flockPtr->boid[i].newPos.x += flockPtr->boid[i].newDir.x * flockPtr->boid[i].speed * (flockPtr->speedupFactor / 100.0);
		flockPtr->boid[i].newPos.y += flockPtr->boid[i].newDir.y * flockPtr->boid[i].speed * (flockPtr->speedupFactor / 100.0);
		flockPtr->boid[i].newPos.z += flockPtr->boid[i].newDir.z * flockPtr->boid[i].speed * (flockPtr->speedupFactor / 100.0);

	}
}

Point3d FindFlockCenter(FlockPtr flockPtr)
{
	double			totalH = 0, totalV = 0, totalD = 0;
	Point3d			centerPoint;
	register short	i;

	for (i = 0 ; i <  flockPtr->numBoids; i++)
	{
		totalH += flockPtr->boid[i].oldPos.x;
		totalV += flockPtr->boid[i].oldPos.y;
		totalD += flockPtr->boid[i].oldPos.z;
	}
	centerPoint.x = (double)	(totalH / flockPtr->numBoids);
	centerPoint.y = (double)	(totalV / flockPtr->numBoids);
	centerPoint.z = (double)	(totalD / flockPtr->numBoids);
		
	return(centerPoint);
}

float MatchAndAvoidNeighbors(FlockPtr flockPtr, short theBoid, Velocity *matchNeighborVel, Velocity *avoidNeighborVel)
{
	short			i, j, neighbor;
	double			distSqr;
	double			dist, distH, distV,distD;
	double			tempSpeed;
	short			numClose = 0;
	Velocity		totalVel = {0.0,0.0,0.0};

	/**********************/
	/* Find the neighbors */	
	/**********************/

	/* special case of one neighbor */
	if (flockPtr->numNeighbors == 1) {
		flockPtr->boid[theBoid].neighborDistSqr[0] = kMaxLong;
	
		for (i = 0; i < flockPtr->numBoids; i++) {
			if (i != theBoid) {
				distSqr = DistSqrToPt(flockPtr->boid[theBoid].oldPos, flockPtr->boid[i].oldPos);
				
				/* if this one is closer than the closest so far, then remember it */
				if (flockPtr->boid[theBoid].neighborDistSqr[0] > distSqr) {
					flockPtr->boid[theBoid].neighborDistSqr[0] = distSqr;
					flockPtr->boid[theBoid].neighbor[0] = i;
				}
			}
		}
	}
	/* more than one neighbor */
	else {
		for (j = 0; j < flockPtr->numNeighbors; j++)
			flockPtr->boid[theBoid].neighborDistSqr[j] = kMaxLong;
		
		for (i = 0 ; i < flockPtr->numBoids; i++) {
			/* if this one is not me... */
			if (i != theBoid) {
				distSqr = DistSqrToPt(flockPtr->boid[theBoid].oldPos, flockPtr->boid[i].oldPos);
	
				/* if distSqr is less than the distance at the bottom of the array, sort into array */
				if (distSqr < flockPtr->boid[theBoid].neighborDistSqr[flockPtr->numNeighbors-1]) {
					j = flockPtr->numNeighbors - 1;
				
					/* sort distSqr in to keep array in size order, smallest first */
					while ((distSqr < flockPtr->boid[theBoid].neighborDistSqr[j-1]) && (j > 0)) {
						flockPtr->boid[theBoid].neighborDistSqr[j] = flockPtr->boid[theBoid].neighborDistSqr[j - 1];
						flockPtr->boid[theBoid].neighbor[j] = flockPtr->boid[theBoid].neighbor[j - 1];
						j--;
					}
					flockPtr->boid[theBoid].neighborDistSqr[j] = distSqr;
					flockPtr->boid[theBoid].neighbor[j] = i;					
				}
			}
		}
	}

	/*********************************/
	/* Match and avoid the neighbors */	
	/*********************************/

	matchNeighborVel->x = 0;
	matchNeighborVel->y = 0;
	matchNeighborVel->z = 0;
	
	// set tempSpeed to old speed
	tempSpeed = flockPtr->boid[theBoid].speed;
	
	for (i = 0; i < flockPtr->numNeighbors; i++) {
		neighbor = flockPtr->boid[theBoid].neighbor[i];
		
		// calculate matchNeighborVel by averaging the neighbor velocities
		matchNeighborVel->x += flockPtr->boid[neighbor].oldDir.x;
		matchNeighborVel->y += flockPtr->boid[neighbor].oldDir.y;
		matchNeighborVel->z += flockPtr->boid[neighbor].oldDir.z;
			
		// if distance is less than preferred distance, then neighbor influences boid
		distSqr = flockPtr->boid[theBoid].neighborDistSqr[i];
		if (distSqr < flockPtr->prefDistSqr) {
			dist = sqrt(distSqr);

			distH = flockPtr->boid[neighbor].oldPos.x - flockPtr->boid[theBoid].oldPos.x;
			distV = flockPtr->boid[neighbor].oldPos.y - flockPtr->boid[theBoid].oldPos.y;
			distD = flockPtr->boid[neighbor].oldPos.z - flockPtr->boid[theBoid].oldPos.z;
			
			if(dist == 0.0) dist = 0.0000001;
			totalVel.x = totalVel.x - distH - (distH * ((float) flockPtr->prefDist / (dist)));
			totalVel.y = totalVel.y - distV - (distV * ((float) flockPtr->prefDist / (dist)));
			totalVel.z = totalVel.z - distD - (distV * ((float) flockPtr->prefDist / (dist)));
		
			numClose++;
		}
		if (InFront(&(flockPtr->boid[theBoid]), &(flockPtr->boid[neighbor]))) {	// adjust speed
			if (distSqr < flockPtr->prefDistSqr) 
				tempSpeed /= (flockPtr->accelFactor / 100.0);
			else
				tempSpeed *= (flockPtr->accelFactor / 100.0);
		}
		else {
			if (distSqr < flockPtr->prefDistSqr)
				tempSpeed *= (flockPtr->accelFactor / 100.0);
			else
				tempSpeed /= (flockPtr->accelFactor / 100.0);
		}
	}
	if (numClose) {
		avoidNeighborVel->x = totalVel.x / numClose;
		avoidNeighborVel->y = totalVel.y / numClose;
		avoidNeighborVel->z = totalVel.z / numClose;
		NormalizeVelocity(matchNeighborVel);
	}
	else {
		avoidNeighborVel->x = 0;
		avoidNeighborVel->y = 0;
		avoidNeighborVel->z = 0;
	}
	return(tempSpeed);
}


Velocity SeekPoint(FlockPtr flockPtr, short theBoid, Point3d seekPt)
{
	Velocity	tempDir;
	tempDir.x = seekPt.x - flockPtr->boid[theBoid].oldPos.x;	
	tempDir.y = seekPt.y - flockPtr->boid[theBoid].oldPos.y;
	tempDir.z = seekPt.z - flockPtr->boid[theBoid].oldPos.z;
	NormalizeVelocity(&tempDir);
	return(tempDir);
}


Velocity AvoidWalls(FlockPtr flockPtr, short theBoid)
{
	Point3d		testPoint;
	Velocity	tempVel = {0.0, 0.0, 0.0};
		
	/* calculate test point in front of the nose of the boid */
	/* distance depends on the boid's speed and the avoid edge constant */
	testPoint.x = flockPtr->boid[theBoid].oldPos.x + flockPtr->boid[theBoid].oldDir.x * flockPtr->boid[theBoid].speed * flockPtr->edgeDist;
	testPoint.y = flockPtr->boid[theBoid].oldPos.y + flockPtr->boid[theBoid].oldDir.y * flockPtr->boid[theBoid].speed * flockPtr->edgeDist;
	testPoint.z = flockPtr->boid[theBoid].oldPos.z + flockPtr->boid[theBoid].oldDir.z * flockPtr->boid[theBoid].speed * flockPtr->edgeDist;

	/* if test point is out of the left (right) side of flockPtr->flyRect, */
	/* return a positive (negative) horizontal velocity component */
	if (testPoint.x < flockPtr->flyRect.left)
		tempVel.x = fabs(flockPtr->boid[theBoid].oldDir.x);
	else if (testPoint.x > flockPtr->flyRect.right)
		tempVel.x = - fabs(flockPtr->boid[theBoid].oldDir.x);

	/* same with top and bottom */
	if (testPoint.y < flockPtr->flyRect.top)
		tempVel.y = fabs(flockPtr->boid[theBoid].oldDir.y);
	else if (testPoint.y > flockPtr->flyRect.bottom)
		tempVel.y = - fabs(flockPtr->boid[theBoid].oldDir.y);

	/* same with front and back*/
	if (testPoint.z < flockPtr->flyRect.front)
		tempVel.z = fabs(flockPtr->boid[theBoid].oldDir.z);
	else if (testPoint.z > flockPtr->flyRect.back)
		tempVel.z = - fabs(flockPtr->boid[theBoid].oldDir.z);

	
	return(tempVel);
}


Boolean InFront(BoidPtr theBoid, BoidPtr neighbor)
{
	float	grad, intercept;
	Boolean result;
	
/* we do this on 2 planes, xy, yz. if one returns false then we know its behind. a.sier/jasch 08/2005

Find the gradient and y-intercept of a line passing through theBoid's oldPos
perpendicular to its direction of motion.  Another boid is in front of theBoid
if it is to the right or left of this linedepending on whether theBoid is moving
right or left.  However, if theBoid is travelling vertically then just compare
their vertical coordinates.	

*/
	// xy plane
	
	// if theBoid is not travelling vertically...
	if (theBoid->oldDir.x != 0) {
		// calculate gradient of a line _perpendicular_ to its direction (hence the minus)
		grad = -theBoid->oldDir.y / theBoid->oldDir.x;
		
		// calculate where this line hits the y axis (from y = mx + c)
		intercept = theBoid->oldPos.y - (grad * theBoid->oldPos.x);

		/* compare the horizontal position of the neighbor boid with */
		/* the point on the line that has its vertical coordinate */
		if (neighbor->oldPos.x >= ((neighbor->oldPos.y - intercept) / grad)) {
			/* return true if the first boid's horizontal movement is +ve */
			result = (theBoid->oldDir.x > 0);

			if (result==0) return 0;
			else goto next;
			
		} else {
			/* return true if the first boid's horizontal movement is +ve */
			result = (theBoid->oldDir.x < 0);
			if (result==0) return 0;
			else goto next;
		}
	}
	/* else theBoid is travelling vertically, so just compare vertical coordinates */
	else if (theBoid->oldDir.y > 0) {
		result = (neighbor->oldPos.y > theBoid->oldPos.y);
		if (result==0){ 
			return 0;
		}else{
			goto next;
		}
	}else{
		result = (neighbor->oldPos.y < theBoid->oldPos.y);
		if (result==0){
			return 0;
		} else {
			goto next;
		}
	}
next:

	// yz plane
	
	// if theBoid is not travelling vertically... 
	if (theBoid->oldDir.y != 0) {
		// calculate gradient of a line _perpendicular_ to its direction (hence the minus) 
		grad = -theBoid->oldDir.z / theBoid->oldDir.y;
		
		// calculate where this line hits the y axis (from y = mx + c) 
		intercept = theBoid->oldPos.z - (grad * theBoid->oldPos.y);

		// compare the horizontal position of the neighbor boid with 
		// the point on the line that has its vertical coordinate 
		if (neighbor->oldPos.y >= ((neighbor->oldPos.z - intercept) / grad)) {
			// return true if the first boid's horizontal movement is +ve 
			result = (theBoid->oldDir.y > 0);
			if (result==0){ 
				return 0;
			}else{
				goto next2;
			}
		} else {
			// return true if the first boid's horizontal movement is +ve 
			result = (theBoid->oldDir.y < 0);
			if (result==0){
				return 0;
			}else{
				goto next2;
			}
		}
	}
	// else theBoid is travelling vertically, so just compare vertical coordinates 
	else if (theBoid->oldDir.z > 0) {
		result = (neighbor->oldPos.z > theBoid->oldPos.z);
		if (result==0){ 
			return 0;
		}else{
			goto next2;
		}
	}else{
		result = (neighbor->oldPos.z < theBoid->oldPos.z);
		if (result==0){
			return 0;
		}else{
			goto next2;
		}
	}
next2:
	return 1;
}

void NormalizeVelocity(Velocity *direction)
{
	float	hypot;
	
	hypot = sqrt(direction->x * direction->x + direction->y * direction->y + direction->z * direction->z );

	if (hypot != 0.0) {
		direction->x = direction->x / hypot;
		direction->y = direction->y / hypot;
		direction->z = direction->z / hypot;
	}
}

double RandomInt(double minRange, double maxRange)
{
	long	qdRdm;
	double	t, result;
	
	qdRdm = rand();
	t = (double)qdRdm / 32768.0; 	// now 0 <= t <= 1
	result = (t * (maxRange - minRange)) + minRange;
	return(result);
}

double DistSqrToPt(Point3d firstPoint, Point3d secondPoint)
{
	double	a, b,c;
	a = firstPoint.x - secondPoint.x;
	b = firstPoint.y - secondPoint.y;	
	c = firstPoint.z - secondPoint.z;	
	return(a * a + b * b + c * c);
}