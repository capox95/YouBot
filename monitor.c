/*
 ============================================================================
 Name        : monitor-template.c
 Author      : Paolo Torroni
 Version     : Nov 20, 2017
 Copyright   : Use as you wish
 Description : Template for the implementation of a monitor and its animation
                using fictitious thread activities
 ============================================================================
 */

/*

1. Identify entities:
    - resources (types, capacity, ...) to be managed by this particular monitor
    - threads (types, number, ...) using such resources
2. Define the monitor’s API
    - method names
    - return values
    - arguments
3. Identify the priorities in the problem
    - among threads
    - among requests
    - ...
4. Define the queues needed to manage the operation of the monitor
    - which queues?
    - which policies inside/between queues?
5. Define the data needed to model the state of the monitor
    - state variables such as integers/counters, boolean variables, and enumeration to model the state
6. Provide a detailed design of methods
    6.1. Identify alternative cases within method
    - if appropriate, define different behaviors based on thread types, resource types, action types, etc.
    6.2. Identify conditions preventing resource acquisition
    6.3. Define signaling/broadcasting upon resource release
    6.4. Define signaling/broadcasting upon resource acquisition
    6.5. Define what the initial state of the monitor should be (initialization)
7. Implement the monitor
    7.1. Start from generic template with the basic #includes, main loop, etc. (see this template), such as:
        7.1.1. Data types for readability (e.g.: typedef thread_name_t char[10], ...)
            - One data type will be the monitor itself (monitor_t)
            - A field of monitor_t will be a mutex lock (pthread_mutex_t), to be used to guard access to the monitor's methods
            - The monitor should also include one condvar per each queue
        7.1.2. Constants, defined for readability
            - e.g.: #define N_THREADS 5
        7.1.3. Global data structures (such as the monitor itself)
        7.1.4. Thread vectors, for the purpose of animating the application and validating the monitor implementation 
            - e.g.: thread_name_t names[N_THREADS], pthread_t my_thread_names[N_THREADS]
    7.2. Declare the monitor APIs. Typically:
        - int init(monitor_t*)
        - int destroy(monitor_t*)
        - int acquire(monitor_t*, ...)
        - int release(monitor_t*, ...)
    7.3. Set up the environment for animating the application and validating the correct behavior of the monitor
        7.3.1. Define the main() function. The typical sequence is:
            - initialize the monitor
            - loop to create N_THREADS threads
            - loop to join N_THREADS threads
            - destroy the monitor
        7.3.2. Define the code associated with each thread type. The typical sequence is:
            FOREVER
            {
                - acquire resource using a monitor method
                - do something with the resource (or waste some time, in a simulated environment)
                - release resource using a monitor method
                - do something without the resource (or waste some more time, in a simulated environment)
            }
        7.3.3. Define all other methods and auxiliary functions (e.g. left(i), right(i), display_state(), eat(), …)
*/

// TEMPLATE FOR STEP 7.0 //

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <sys/types.h>

// CONSTANTS AND MACROS
// for readability
#define N_THREADS 5
#define FOREVER for(;;)

// DEFINITIONS OF NEW DATA TYPES
// for readability
typedef char thread_name_t[10];
typedef enum {STATE1, STATE2, STATE3} state_t;
 // monitor also defined as a new data type
typedef struct {
    // monitor lock (one mutex lock per monitor)
    pthread_mutex_t m;
    // condvars (one condvar per queue)
    // best if give meaningful names instead of cv_1, cv_2, ...
    pthread_cond_t condvar_1, condvar_2, condvar_3, ...;
    // state variables
    int n_w1, n_w2, ..., counter, ...;
    state_t statevar, ...;
} monitor_t;

// GLOBAL VARIABLES
// the monitor should be defined as a global variable
monitor_t mon;


//  MONITOR API
void method_acquire(monitor_t *mon, char* name, ...);
void method_release(monitor_t *mon, char* name, ...);
void monitor_init(monitor_t *mon);
void monitor_destroy(monitor_t *mon);

// OTHER FUNCTION DECLARATIONS
// functions corresponding to thread entry points
void *thread_type_1(void *arg); 
// spend_some_time could be useful to waste an unknown amount of CPU cycles, up to a given top 
double spend_some_time(int);
// simulate_action_* to be replaced by function names according to application: e.g., pick_up, put_down, ...
void simulate_action_1() { spend_some_time(100); }
void simulate_action_2() { spend_some_time(2); }
void simulate_action_3() { spend_some_time(10); }
...

// IMPLEMENTATION OF MONITOR API
void method_acquire(monitor_t *mon, char *name, ...) {
	pthread_mutex_lock(&mon->m);
		if( /* <case_1> */ ) {
			// update state
			while(!( <condition_1> )) {
				pthread_cond_wait(&mon->condvar_1,&mon->m);
			}
			// update state after resource acquisition
			// signal/broadcast after acquisition
		}
		else if( /* <case_2> */ ) {
			// update state
			while(!( <condition_2> )) {
				pthread_cond_wait(&mon->condvar_2,&mon->m);
			}
			// update state after resource acquisition
			// signal/broadcast after acquisition
		}
		else if( /* <case_3> */ ) {
			// update state
			while(!( <condition_3> )) {
				pthread_cond_wait(&mon->condvar_3,&mon->m);
			}
			// update state after resource acquisition
			// signal/broadcast after acquisition
		}
		else {
			...
		}
	pthread_mutex_unlock(&mon->m);
}

void resource_release(monitor_t *mon, char *name, ...) {
	pthread_mutex_lock(&mon->m);
		if( /* <case_1> */ ) {
			// update state
			// signal/broadcast after release
		}
		else if( /* <case_2> */ ) {
			// update state
			// signal/broadcast after release
		}
		else ...
	pthread_mutex_unlock(&mon->m);
}

void monitor_init(monitor_t *mon) {
	// set initial value of monitor data structures, state variables, mutexes, counters, etc.
    // typically can use default attributes for monitor mutex and condvars
    pthread_mutex_init(&mon->m,NULL);
    pthread_cond_init(&mon->condvar_1,NULL);
    pthread_cond_init(&mon->condvar_2,NULL);
    ...
    // set all condvar counters to 0
    mon->n_w1=0;
    mon->n_w2=0;
    ...
    // initialize whatever other structures
    mon->counter=...;
}

void monitor_destroy(monitor_t *mon) {
    // set initial value of monitor data structures, state variables, mutexes, counters, etc.
    pthread_cond_destroy(&mon->condvar_1);
    pthread_cond_destroy(&mon->condvar_2);
    ...
    pthread_mutex_destroy(&mon->m);
}

// MAIN FUNCTION
int main(void) {
    // thread management data structures
    pthread_t my_threads[N_THREADS];
    thread_name_t my_thread_names[N_THREADS];
    int i;

    // initialize monitor data strcture before creating the threads
	monitor_init(&mon);

    for (i=0;i<N_THREADS;i++) {
     	sprintf(my_thread_names[i],"t%d",i);
        // create N_THREADS thread with same entry point 
        // these threads are distinguishable thanks to their argument (their name: "t1", "t2", ...)
        // thread names can also be used inside threads to show output messages
        pthread_create(&my_threads[i], NULL, thread_type_1, my_thread_names[i]);
    }

    for (i=0;i<N_THREADS;i++) {
        pthread_join(my_threads[i], NULL);
    }

    // free OS resources occupied by the monitor after creating the threads
    monitor_destroy(&mon);

    return EXIT_SUCCESS;
}

// TYPE 1 THREAD LOOP
void *thread_type_1(void *arg) {
	// local variables definition and initialization
	char *thread_name = ...; // thread_name = f(arg)
	FOREVER { // or any number of times
		simulate_action_1();

		method_acquire(&mon, thread_name, ...);
		simulate_action_2();
		method_release(&mon, thread_name, ...);

		simulate_action_3();

		method_acquire(&mon, thread_name, ...);
		simulate_action_4();
		method_release(&mon, thread_name, ...);
	}
	pthread_exit(NULL);
}

// AUXILIARY FUNCTIONS
double spend_some_time(int max_steps) {
    double x, sum=0.0, step;
    long i, N_STEPS=rand()%(max_steps*1000000);
    step = 1/(double)N_STEPS;
    for(i=0; i<N_STEPS; i++) {
        x = (i+0.5)*step;
        sum+=4.0/(1.0+x*x);
    }
    return step*sum;
}
