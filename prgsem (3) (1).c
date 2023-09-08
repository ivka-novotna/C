#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <termios.h> 
#include <unistd.h>
#include <pthread.h>
#include "prg_serial_nonblock.h"
#include "messages.h"
#include "event_queue.h"
//#include <png.h>
#include "xwin_sdl.h"

#define SERIAL_READ_TIMOUT_MS 500
#define def_chunksize 20
#define maxchunksize 250
#define RGB 3
#define SDL_EVENT_POLL_WAIT_MS 10
#define ERROR_MALLOC 101
#define ERROR_ASSERT 105

typedef struct {
   bool quit;
   int fd;
} data_t;
static struct{
    double c_re;
    double c_im;
    int n;

    double range_re_min;
    double range_re_max;
    double range_im_min;
    double range_im_max;

    int grid_w;
    int grid_h;

    int cur_x;
    int cur_y;

    double d_re;
    double d_im;

    int nbr_chunks;
    int cid;
    double chunk_re;
    double chunk_im;

    uint8_t chunk_n_re;
    uint8_t chunk_n_im;

    uint8_t *grid;
    bool computing;
    bool done;
	bool computation_set;
	bool first;
	bool abort;
} comp ={
    .c_re = -0.4,
    .c_im = 0.6,

    .n = 60,

    .range_re_min = -1.6,
    .range_re_max = 1.6,
    .range_im_min = -1.1,
    .range_im_max = 1.1,
    
    .grid = NULL,
    .grid_w = 640,
    .grid_h = 480,

    .chunk_n_re = 64,
    .chunk_n_im = 48,

    .computing = false,
    .done = false,
	.computation_set = true,
	.first = false,
	.abort = false
};    

static struct{
    int w;
    int h;
    unsigned char *img;
} gui = {.img = NULL};


pthread_mutex_t mtx;
pthread_cond_t cond;

void my_assert(bool r, const char *fcname, int line, const char *fname);
void* keyboard_thread(void* d);
void* main_thread(void* d);
void call_termios(int reset);
bool send_message(data_t *data, message *msg); 
void computation_init(void);
void computation_cleanup(void);
bool set_compute(message *msg);
bool compute(message *msg);
bool compute_on_pc(unsigned char *img);
bool is_computing(void);
bool is_done(void);
void abort_comp(void);
void update_image(int w, int h, unsigned char *img);
void get_grid_size(int *w, int *h);
void gui_init(void);
void gui_cleanup(void);
void gui_refresh(void);
void* my_alloc(size_t size);
void update_data(const msg_compute_data *compute_data);


int main(int argc, char *argv[]){

	data_t data = { .quit = false, .fd = -1};
	const char *serial = argc > 1 ? argv[1] : "/dev/ttyACM0";
	data.fd = serial_open(serial);
	if (data.fd == -1) {
		fprintf(stderr, "ERROR: Cannot open serial port %s\n", serial);
		exit(100);
	}
	pthread_mutex_init(&mtx, NULL); // initialize mutex with default attributes
	pthread_cond_init(&cond, NULL); // initialize mutex with default attributes
	call_termios(0);
	enum { KEYBOARD_THREAD, MAIN_THREAD, NUM_THREADS };
	const char *threads_names[] = { "Keyboard", "Main" };
	void* (*thr_functions[])(void*) = {keyboard_thread, main_thread};
	pthread_t threads[NUM_THREADS];

	for (int i = 0; i < NUM_THREADS; ++i) {
		int r = pthread_create(&threads[i], NULL, thr_functions[i], &data);
		fprintf(stderr, "INFO: Create thread '%s' %s\n", threads_names[i], ( r == 0 ? "OK" : "FAIL") );
	}
	fprintf(stderr, "INFO: Use 'o' key to see options\n");
	message msg;
	bool quit = false, fast = false; 
	computation_init();
	gui_init();
	while(!quit){
		event ev = queue_pop(); // get first event in queue
		if(ev.source == EV_KEYBOARD){
			msg.type = MSG_NBR; 
			if (comp.abort)
				comp.abort = false;
			switch(ev.type){
				case EV_GET_VERSION: // version
					msg.type = MSG_GET_VERSION;
					fprintf(stderr, "INFO: Get version requested\n");
					break;
				case EV_CLEAR_BUFFER: // clean buffer
					if (is_computing()) //calculations are in progress
						fprintf(stderr,"WARN: Clear buffer request discarderd, its computing\n\r");
					else{
						gui_cleanup();
						gui_init();
						comp.cid = -1;
						fprintf(stderr,"INFO: Buffer was cleared, chunkid is restarted\n\r");
					}
					break;
				case EV_RESET_CHUNK: // reset chunk
					if (comp.computing) // calculations are in progress
						fprintf(stderr, "WARN: Chunk reset request discarded, it is currently computing\n\r");
					else{ // ok
						comp.cid = -1;
						fprintf(stderr, "INFO: Chunk reset request\n\r");
					}
					break;
				case EV_ABORT: // interupt
					if (!comp.computing) // calculations are NOT in progress
						fprintf(stderr, "WARN: Abort requested but it is not computing\n\r");	
					else{ // calculations are in progress
						msg.type = MSG_ABORT;
						comp.computing = false;
						comp.abort = true;
						comp.cid--;
						fprintf(stderr, "INFO: Abort from Computer\n\r");
						
					}
					break;
				case EV_SET_COMPUTE: // fixing parameters
					msg.type = MSG_SET_COMPUTE;
					if(set_compute(&msg)){
						fprintf(stderr,"INFO: Computation parameters has been set up\n\r");
					}
					else
						fprintf(stderr,"WARN: New computation parameters requested but it is discarded due to on ongoing computation\n");
					break;
				case EV_COMPUTE: // calculations on Nucleo
					if (comp.computing) // calculations are in progress
						fprintf(stderr, "WARN: New computation requested but it is discarded due on ongoing computation\n\r");
					else if (!comp.computation_set) // parameters not given, set_compute 
						fprintf(stderr, "Computation was requested, but parameters has not been set up yet, if you want to set up parameters press 's'\n\r");
					else{ // ok
						msg.type = MSG_COMPUTE;
						compute(&msg);
						fprintf(stderr,"INFO: New computation chunk id: %d for part %d x %d\n", msg.data.compute.cid, msg.data.compute.n_re, msg.data.compute.n_im);
					}
					break;
				case EV_COMPUTE_CPU: // computing on a computer
					if (!comp.computation_set)
						fprintf(stderr, "WARN: Computation was requested, but parameters has not been set up yet\n\r");
					else{
						compute_on_pc(gui.img);
						fprintf(stderr, "INFO: Computation was provided on computer\n\r");
					}
					break;
				case EV_REFRESH: // update image
					xwin_close(); //// update the image after resizing or just close, clear, initialize, black screen, redraw
					gui_cleanup();
					gui_init();
					gui_refresh();
					comp.cid = -1;
					fprintf(stderr, "INFO: Screen has been refreshed\r\n");
					break;
				case EV_ENHANCE_BAUD: // baud rate change
					if (comp.computing) // calculations are in progress
						fprintf(stderr, "WARN: Nucleo is able to save image only when it is not computing\r\n");
					else{ //ok
						msg.type = MSG_ENHANCE_BAUD;
						fprintf(stderr, "INFO: Baud rate has been enhanced\r\n");
					}
					break;
				case EV_QUIT:
					quit = true;
					data.quit = quit;
					break;
				default:
					break;
			}// end switch
			if(msg.type != MSG_NBR){ // message was sent successfully
				if (!send_message(&data, &msg))
					fprintf(stderr, "ERROR: send_message() does not send all bytes of the message!\n");
				if (msg.type == MSG_ENHANCE_BAUD){ // request to change baud rate
					pthread_mutex_lock(&mtx); 
					if (fast){ 
						data.fd = serial_open("/dev/ttyACM0"); 
						fast = false;
					}
					else if(!fast){
						data.fd = serial_open_bigratemode("/dev/ttyACM0");
						fast = true;
					}
					pthread_mutex_unlock(&mtx);
				}	
			}
		} else if (ev.source == EV_NUCLEO) { // handle nucleo events
			if (ev.type == EV_SERIAL) {
				message *msg = ev.data.msg;  // record a message
				switch (msg->type) {
					case MSG_STARTUP: // launch
					{
						char str[STARTUP_MSG_LEN + 1];
						for (int i = 0; i < STARTUP_MSG_LEN; ++i)
							str[i] = msg->data.startup.message[i];
						str[STARTUP_MSG_LEN] = '\0';
						comp.computing = false; //restart variables
						comp.computation_set = false;
						comp.cid = -1;
						fprintf(stderr, "INFO: Nucleo restarted - '%s'\n", str);
						break;
					}
					case MSG_VERSION: // get version
						if (msg->data.version.patch > 0) 
							fprintf(stderr, "INFO: Nucleo firmware ver. %d.%d-p%d\n", msg->data.version.major, msg->data.version.minor, msg->data.version.patch);
						else
							fprintf(stderr, "INFO: Nucleo firmware ver. %d.%d\n", msg->data.version.major, msg->data.version.minor);
						break;
					case MSG_ERROR: // error
						fprintf(stderr, "WARN: Receive error from Nucleo\r\n");
						break;
					case MSG_OK: // ок)
						fprintf(stderr, "INFO: Receive ok from Nucleo\r\n");
						break;
					case MSG_ABORT: // interrupt
						fprintf(stderr, "INFO: Abort from Nucleo\r\n");
						if (!comp.computing) // no calculations in progress
							fprintf(stderr, "WARN: Abort from NUCLEO is requested but it is not computing\n\r");	
						else{
							comp.computing = false;
							comp.cid--;
						}
						break;
					case MSG_DONE: // executed
						gui_refresh();
						if (!is_done()){ // report of successful calculations
							comp.computing = false;
							fprintf(stderr, "INFO: Nucleo sent a chunk %d\n",comp.cid);
							if (comp.cid != comp.nbr_chunks){
								event ev = {.source = EV_KEYBOARD, .type = EV_COMPUTE};
								queue_push(ev);
								fprintf(stderr, "INFO: Sending request for another chunk to nucleo\n");
							}
							else{
								comp.computing = false;
								fprintf(stderr, "INFO: Nucleo has computed the image\r\n");
							}
						}
						else {// calculations still in progress, request to interupt
							fprintf(stderr, "INFO: Abort from computer\r\n");
						}
						break;
					case MSG_COMPUTE_DATA: // calculations and redrawing
						if(comp.computing){
							if (comp.cid != msg->data.compute_data.cid){
								fprintf(stderr,"WARN: received compute data has chunk id %d which is different from chunk id %d - cannot align data to the grid properly\x0a\r\n", msg->data.compute_data.cid, comp.cid);
							} else{
								update_data(&(msg->data.compute_data));
							}
					
						}else
							if(!comp.abort) 
							fprintf(stderr, "WARN: Nucleo sends new data without computing \r\n");
						break;
					default:
						break;			
				} // end switch
			} //end if(ev.type == EV.SERIAL)
		} // end else if (ev.source == EV_NUCLEO)
	} //end while(!quit)
    queue_cleanup(); // cleanup all events and free allocated memory for messages.
	computation_cleanup();
	gui_cleanup();
   	for(int i = 0; i < NUM_THREADS; ++i){
		fprintf(stderr, "INFO: Call join to the thread %s\n", threads_names[i]);
		int r = pthread_join(threads[i], NULL);
		fprintf(stderr, "INFO: Joining the thread %s has been %s\n", threads_names[i], (r == 0 ? "OK" : "FAIL"));
   	}
	serial_close(data.fd);
	call_termios(1); // restore terminal settings
	return EXIT_SUCCESS;
} // end of main

void call_termios(int reset)
{
    static struct termios tio, tioOld;
    tcgetattr(STDIN_FILENO, &tio);
    if (reset) {
        tcsetattr(STDIN_FILENO, TCSANOW, &tioOld);
    } else {
        tioOld = tio; //backup 
        cfmakeraw(&tio);
        tcsetattr(STDIN_FILENO, TCSANOW, &tio);
    }
}

bool send_message(data_t *data, message *msg) {
  	pthread_mutex_lock(&mtx);
	int fd = data->fd;
	pthread_mutex_unlock(&mtx);
	int size = sizeof(message), length;
	get_message_size(msg->type, &length);
	uint8_t msg_buf[size];
	if(fill_message_buf(msg, msg_buf, size, &length)){
		for (int i = 0; i < length; i++)
			serial_putc(fd, msg_buf[i]);
		return true;
	} 
	return false;
}


void my_assert(bool r, const char *fcname, int line, const char *fname){
    if(!r){
        fprintf(stderr,"ERROR: my_assert FAIL%s() line %d in %s\n", fcname, line, fname);
        exit(ERROR_ASSERT);
    }
}


void update_image(int w, int h, unsigned char *img){
    my_assert(img&& comp.grid && w == comp.grid_w && h == comp.grid_h, __func__, __LINE__, __FILE__);
    for(int i = 0; i< w*h; ++i){
        const double t = 1. * comp.grid[i] / (comp.n + 1.0);
        *(img++) = 9 * (1-t) * t * t *  t *255;
		*(img++) = 15 * (1-t) * (1-t) * t * t *255;
		*(img++) = 8.5 * (1-t) * (1-t) * (1-t) * t *255;
    }
}


void get_grid_size(int *w, int *h){
    *w = comp.grid_w;
    *h = comp.grid_h;    
}

void gui_init(void){
    get_grid_size(&gui.w, &gui.h);
    gui.img = my_alloc(gui.w * gui.h * RGB);
    my_assert(xwin_init(gui.w, gui.h) == 0, __func__, __LINE__, __FILE__);
}

void gui_cleanup(void){
    if (gui.img){
        free(gui.img);
        gui.img = NULL;
    }
    xwin_close();
}

void gui_refresh(void){
    if(gui.img){
        update_image(gui.w, gui.h,gui.img);
        xwin_redraw(gui.w, gui.h,gui.img);
    }
}

void* my_alloc(size_t size){
    void *ret = malloc(size);
    if(!ret){
        fprintf(stderr, "ERROR: cannot malloc!\n");
        exit(ERROR_MALLOC);
    }
	return ret;
}

void computation_init(void){
    comp.grid = my_alloc(comp.grid_w * comp.grid_h);
    comp.d_re = (comp.range_re_max - comp.range_re_min) / (1. *comp.grid_w);
    comp.d_im = -(comp.range_im_max - comp.range_im_min) / (1. *comp.grid_h);
    comp.nbr_chunks = (comp.grid_w * comp.grid_h) / (comp.chunk_n_re * comp.chunk_n_im);

}

void computation_cleanup(void){
    if (comp.grid){
        free(comp.grid);
    }
    comp.grid = NULL;
}

bool is_computing(void){
    return comp.computing;
}

bool is_done(void){
    return comp.done;
}

void abort_comp(void){
    comp.computing = false;
}

bool set_compute(message *msg){
    my_assert(msg != NULL, __func__, __LINE__, __FILE__);
    bool ret = !is_computing();
    if (ret){
		msg->data.set_compute.c_re = comp.c_re;
		msg->data.set_compute.c_im = comp.c_im;
		msg->data.set_compute.d_re = comp.d_re;			
		msg->data.set_compute.d_im = comp.d_im;
 		msg->data.set_compute.n = comp.n;
        comp.done = false;   
    }
    return ret;

}
bool compute(message *msg){
    if(!comp.first){ // first chunk
        comp.cid = 0;
        comp.computing = true;
        comp.cur_x = comp.cur_y = 0; // start computation of the whole image
        comp.chunk_re = comp.range_re_min; //upper - "left" corner
        comp.chunk_im = comp.range_im_max; //"upper" - left corner
        msg->type = MSG_COMPUTE;
		comp.first = true;
    } else { //next chunk
        comp.computing = true;
        comp.cid += 1;
        if (comp.cid < comp.nbr_chunks){
            comp.cur_x += comp.chunk_n_re;
			comp.chunk_re += comp.chunk_n_re * comp.d_re;
            if (comp.cur_x >= comp.grid_w){
                comp.chunk_re = comp.range_re_min; 
                comp.chunk_im += comp.chunk_n_im * comp.d_im;                
                comp.cur_x = 0;       
                comp.cur_y += comp.chunk_n_im;
            }
            msg->type = MSG_COMPUTE;
        }else {// all has been computed     
			msg->type = MSG_DONE;
        }
    }
    
    if (comp.computing && msg->type == MSG_COMPUTE){
        msg->data.compute.cid = comp.cid;
        msg->data.compute.re = comp.chunk_re;
        msg->data.compute.im = comp.chunk_im;
        msg->data.compute.n_re = comp.chunk_n_re;
        msg->data.compute.n_im = comp.chunk_n_im;

    }

    return is_computing();
}

bool compute_on_pc(unsigned char * img){
    my_assert(img && comp.grid , __func__, __LINE__, __FILE__);
	for (int x = 0; x < comp.grid_w; x++){
		for (int y = 0; y < comp.grid_h; y++){
			double newR =  -0.5 + (comp.grid_w-y)*comp.d_re
			double newI =  -0.5 +  y*comp.d_im;
			double oldR, oldI;
			uint8_t iter = 0;	
			while (iter < comp.n && newR*newR + newI*newI < 4){ //calculate the number of iterations
				oldR = newR;
				oldI = newI;
				newR = oldR * oldR - oldI * oldI + comp.c_re;
				newI = 2 * oldR * oldI + comp.c_im;
				iter++;
			}			
			const double t = 1. * iter / (comp.n + 1.0); // calculate the depth
			*(img++) = 9 * (1-t) * t * t *  t *255;
			*(img++) = 15 * (1-t) * (1-t) * t * t *255;
			*(img++) = 8.5 * (1-t) * (1-t) * (1-t) * t *255;
		}
	}
	xwin_redraw(gui.w, gui.h,gui.img);
	comp.computing = false;
	return comp.computing;
}

void update_data(const msg_compute_data *compute_data){
    my_assert(compute_data != NULL, __func__, __LINE__, __FILE__);
    if (compute_data->cid == comp.cid){
        const int idx = comp.cur_x + compute_data->i_re + (comp.cur_y + compute_data->i_im)* comp.grid_w;
        if(idx >= 0 && idx <(comp.grid_w * comp.grid_h)){
            comp.grid[idx] = compute_data->iter;
        }
        if ((comp.cid + 1) >= comp.nbr_chunks && (compute_data->i_re +1) == comp.chunk_n_re &&(compute_data->i_im +1) == comp.chunk_n_im){
            comp.done = true;
            comp.computing = false;
        }
    } else{
        fprintf(stderr, "WARN: Received chunk with enexpected chunk id(cid) \r\n");

    }

}


void* keyboard_thread(void* d){
   data_t *data = (data_t*)d;
   bool end = false;
   int c;
   event ev = { .source = EV_KEYBOARD };
   while ( !end && (c = getchar())){
        ev.type = EV_TYPE_NUM;
        switch(c) {
            case 'g': // get firmware version of Nucleo program
                ev.type = EV_GET_VERSION;
                break;
            case 's': // set calculation paramters
                ev.type = EV_SET_COMPUTE;
                break;
            case '1': // start calculation
                ev.type = EV_COMPUTE;
                break;
            case 'a': // abort ongoing calculations
                ev.type = EV_ABORT;
                break;
            case 'r': // reset chunk => set to zero
                ev.type = EV_RESET_CHUNK;
                break;
            case 'l': // deletes the current content of the calculation (buffer)
                ev.type = EV_CLEAR_BUFFER;
                break;
            case 'p': // redraws the contents of the window with the current state of the calculation (buffer)
                ev.type = EV_REFRESH;
                break;
            case 'c': // compute fractal on PC (for testing and control purposes)
                ev.type = EV_COMPUTE_CPU;
                break;            
            case 'q': // exit program
                end = true;
                break;
            case 'b': // increase baud rate 
                ev.type = EV_ENHANCE_BAUD;
                break;
            case 'o':
                fprintf(stderr, "||===============__Options__========================||\n\r");
                fprintf(stderr, "||  g => Get firmware version of Nucleo program     ||\n\r");
                fprintf(stderr, "||  s => Set calculation paramters                  ||\n\r");
                fprintf(stderr, "||  1 => Compute on Nucleo                          ||\n\r");
                fprintf(stderr, "||  a => Abort computation                          ||\r\n");
                fprintf(stderr, "||  r => Reset chunks                               ||\r\n");
                fprintf(stderr, "||  l => Reset buffer                               ||\r\n");
                fprintf(stderr, "||  p => Refresh image                              ||\r\n");
                fprintf(stderr, "||  c => Compute on computer                        ||\r\n");
                fprintf(stderr, "||  b => Change baud rate                           ||\r\n");
                fprintf(stderr, "||  q => Exit                                       ||\r\n");
                fprintf(stderr, "||==================================================||\r\n");
                break;            
            default:
                fprintf(stderr, "WARN: Key was not recognised\r\n");
                break;
        }
        if (ev.type != EV_TYPE_NUM)// new event 
            queue_push(ev);
        pthread_mutex_lock(&mtx);
        end = end || data->quit; 
        data->quit = end;
        pthread_mutex_unlock(&mtx);
    }
    ev.type = EV_QUIT;
    queue_push(ev);
    fprintf(stderr, "INFO: Exit input thead %p\n", (void*)pthread_self());
    return NULL;
}
void* main_thread(void* d){
	data_t *data = (data_t*)d;
	uint8_t msg_buf[sizeof(message)]; // maximal buffer for all possible messages defined in messages.h
	event ev = { .source = EV_NUCLEO, .type = EV_SERIAL, .data.msg = NULL };
	unsigned char c; // keyboard command
	message* msg = NULL;
	bool quit = false, loading = false; //end and validation of input
	int size, size_buf = 0;
	
	while (serial_getc_timeout(data->fd, SERIAL_READ_TIMOUT_MS, &c) > 0) {}; // задержка против случайных нажатий
	while (!quit) {
		int r = serial_getc_timeout(data->fd, SERIAL_READ_TIMOUT_MS, &c); // задержка
		if (r > 0) { // cmd is entered
			if(!loading){ 
				if(!get_message_size((uint8_t)c, &size))
					fprintf(stderr, "ERROR: Unknown message type has been received 0x%x\n - '%c'\r", c, c);
				msg_buf[size_buf++] = (uint8_t)c; //cmd buffer
				loading = true; // cmd is written
			}
			else if (loading){ 
				msg_buf[size_buf++] = (uint8_t)c; //cmd buffer 
				if (size_buf == size){
					msg = (message*) malloc(sizeof(message));
					if (parse_message_buf(msg_buf, size, msg)){  
						ev.data.msg = msg;  // write message 
						queue_push(ev);  
						size_buf = 0; // reset size
						loading = false; 
					}
					else // иначе 
						fprintf(stderr, "ERROR: Cannot parse message type %d\n\r", msg_buf[0]);
				}
			}
		} 
		else if(r == 0){  //if the cmd was read but not written
			if (loading){  
				fprintf(stderr, "ERROR: UNHANDELED ERROR!!!");
				loading = false;
				size_buf = 0;   
			}
		} 
		else {  //in any other case
			fprintf(stderr, "ERROR: Cannot receive data from the serial port\n");
			quit = true;
		}   
		pthread_mutex_lock (&mtx);  // block thread for dependent variable
		quit = quit || data->quit;
		pthread_mutex_unlock(&mtx); //unlock thread
   }
   ev.type = EV_QUIT;  // end of stream
   queue_push(ev);
   fprintf(stderr, "INFO: Exit serial_rx_thread %p\n", (void*)pthread_self());
   return NULL;
}
