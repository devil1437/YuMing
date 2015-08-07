/* todo: 
 * 3g receive/send in kernel space 
 * memoery cache
 **/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/poll.h>
#include <sys/time.h>

/*
   0       CPU(non-idle)
   1       CPU(idle)
   2       process Running
   3       softirq
   4       memFree
   5       memCache
   6       Flash Read Issue
   7       Flash Read Sector
   8       Flash Write Issue
   9       Flash Write Sector
   10      SDcard Read Issue
   11      SDcard Read Sector
   12      SDcard Write Issue
   13      SDcard Write Sector
   14      touch interrupt
   15      timer interrupt
   16      3G Receive
   17      3G Send
   18      Wifi Receive
   19      Wifi Send
*/

#define SAMPLE_UNIT_TIME 100000  /* 66666us = 15Hz */
#define SAMPLE_UNIT_SECOND 0
#define RESOURCE_NUMBER 20
#define PATH_QUEUE_SIZE 10
#define MAX_CPU_LEVEL 11
#define MAX_OF_CT_RECENT 5
#define MAX_OF_CT_PAST 5
//#define DEBUG

typedef struct state State;
typedef struct path Path;
typedef struct corTable CorTable;
typedef struct list List;

struct state
{
	/* State */
	int ID;
	int resourceRequestBitmap;
	double meanDuration;
	int accuDuration;
	double meanWorkload;
	int accuWorkload;
	int count;
	State *next;

	/* Path */
	Path *head, *rear;
	int pathSize;
	int pathQueueID[PATH_QUEUE_SIZE];
	int pathQueueTop;
	int pathAccuCount;
};

struct path
{
	int destID;
	int count;
	Path *next;
	CorTable *T;
};

struct corTable
{
	double utilOfRecent[MAX_OF_CT_RECENT];
	double utilOfPast[MAX_OF_CT_PAST];
	int indexOfRecent;
	int indexOfPast;
	int countOfRecent;
	int countOfPast;
	double meanOfRecent;
	double meanOfPast;
};

struct list
{
	int size;
	State *head, *rear, *current;
};

State* insertState(List *, int, int, int);
void checkNewState(List *, double *, double *, int);
void showBitmap(int, int);
State* hasSameBitmap(List *, int);
Path *hasSamePath( Path *, int);
void updatePath( Path *);
CorTable * initCorTable(void);
void updateCorTable(CorTable *, double);
double getValueFromCorTable(CorTable *);
void updateState(State *, int, int, int);
double computeSTD(int *, int);
double computeMEAN(int *, int);
void stat();
void setCPULevel(int);


int freq_table[MAX_CPU_LEVEL]={
	200000,
	500000,
	600000,
	700000,
	800000,
	900000,
	1000000,
	1100000,
	1200000,
	1300000,
	1400000
};

FILE *out;
int data_index = 0;
double data[24000][3];
unsigned long long CPUInfo[10], ctxt, processR, softirq;
unsigned long long memFree, memCache;
unsigned long long systemFlash[11], dataFlash[11], cacheFlash[11], devlogFlash[11], paramFlash[11], radioFlash[11], SDcard[11];
unsigned long long touchIntr, timerIntr, localTimerIntr[4];
unsigned long long Net3GR, Net3GS, NetWifiR, NetWifiS;
/* timer */
int res;
struct itimerval tick ;
int stopFlag = 0;
int curFreqLevel = MAX_CPU_LEVEL-1;
double curData[20], prevData[20];
List stateList;


int main(int argc, char **argv)
{
	int i;
	FILE *fp;

	/* Initial Data */
	for( i = 0 ; i < 20 ; i++ ) curData[i] = prevData[i] = 0;
	stateList.size = 0;
	stateList.head = stateList.rear = stateList.current = NULL;
	setCPULevel(curFreqLevel); //set CPU frequency to maximum available frequency

	/**
	 * Timer interrupt handler
	 */
	signal(SIGALRM, stat) ;             /* SIGALRM handeler */
	/* setting first time interval */
	tick.it_value.tv_sec = SAMPLE_UNIT_SECOND ;
	tick.it_value.tv_usec = SAMPLE_UNIT_TIME ;
	/* setting next time interval */
	tick.it_interval.tv_sec = SAMPLE_UNIT_SECOND ;
	tick.it_interval.tv_usec = SAMPLE_UNIT_TIME ;
	res = setitimer(ITIMER_REAL, &tick, NULL);

#ifdef DEBUG
	State *tmp;
	tmp = stateList.head;
	while( tmp != NULL ){ /* for each state */
		/*
		   if( tmp->count > 0 )
		   {
		   printf("id =%3d count =%4d duration =%5.2f workload =%5.2f r = ", tmp->ID, tmp->count, tmp->meanDuration, tmp->meanWorkload);
		   showBitmap( tmp->resourceRequestBitmap, RESOURCE_NUMBER );
		   }
		   */
		printf("====================================\n");
		printf("pathSize = %d, pathQueueTop = %d, pathAccuCount = %d\n", tmp->pathSize, tmp->pathQueueTop, tmp->pathAccuCount);

		int i;
		for( i = 0 ; i < PATH_QUEUE_SIZE ; i++ ) printf("%d ", tmp->pathQueueID[i]);
		printf("\n");

		Path *tmp1;
		tmp1 = tmp->head;
		while( tmp1 != NULL )
		{
			printf("destID = %d, count = %d, p = %.2f\n", tmp1->destID, tmp1->count, (double)tmp1->count/tmp->pathAccuCount);
			printf("mean of recent = %.2f, accu = %d\n", tmp1->T->meanOfRecent, tmp1->T->countOfRecent);
			for( i = 0 ; i < MAX_OF_CT_RECENT ; i++ ) printf("%.1f ", tmp1->T->utilOfRecent[i]);
			printf("\n");
			printf("mean of past = %.2f, accu = %d\n", tmp1->T->meanOfPast, tmp1->T->countOfPast);
			for( i = 0 ; i < MAX_OF_CT_PAST ; i++ ) printf("%.2f ", tmp1->T->utilOfPast[i]);
			printf("\n");

			tmp1 = tmp1->next;
		}

		tmp = tmp->next;
	}
#endif

	/* free list ....................................*/

	/* */
	while(1)
	{   
		char c;
		c = getchar();
		if( c == 'p' ) 
		{   
			stopFlag = 1;
			setCPULevel(MAX_CPU_LEVEL - 1);
			break;
		}   
	}   
	while( stopFlag != 2 );

	return 0;
}

State* insertState(List *list, int bitmap, int startDuration, int startWorkload)
{
	State *newNode;
	newNode = (State *) malloc(sizeof(State));
	newNode->ID = list->size;
	newNode->resourceRequestBitmap = bitmap;
	newNode->meanDuration = 0;
	newNode->accuDuration = startDuration;
	newNode->meanWorkload = 0;
	newNode->accuWorkload = startWorkload;

	newNode->count = 1;
	newNode->next = NULL;
	newNode->head = NULL;
	newNode->rear = NULL;
	newNode->pathSize = 0;
	newNode->pathQueueTop = 0;
	newNode->pathAccuCount = 0;

	list->size++;
	if(list->head == NULL) list->head = newNode;
	else list->rear->next = newNode;
	list->rear = newNode;

	return newNode;
}

void checkNewState(List *list, double *curData, double *prevData, int num)
{
	int resourceRequestBitmap;
	resourceRequestBitmap = 0;

	/* not use forloop, due to the different granularity*/
	/* Number of Running Process */
	resourceRequestBitmap = ((curData[2] - prevData[2]) > 0) ? (resourceRequestBitmap | (1 << 2)) : resourceRequestBitmap;
	/* Number of Softirq */
	resourceRequestBitmap = ((curData[3] - prevData[3]) > 0) ? (resourceRequestBitmap | (1 << 3)) : resourceRequestBitmap;
	/* memFree */
	resourceRequestBitmap = ((curData[4] - prevData[4]) < 0) ? (resourceRequestBitmap | (1 << 4)) : resourceRequestBitmap;
	/* memCache */
	resourceRequestBitmap = ((curData[5] - prevData[5]) > 0) ? (resourceRequestBitmap | (1 << 5)) : resourceRequestBitmap;
	/* Flash Read Issue */
	resourceRequestBitmap = ((curData[6] - prevData[6]) > 0) ? (resourceRequestBitmap | (1 << 6)) : resourceRequestBitmap;
	/* Flash Read Sector */
	resourceRequestBitmap = ((curData[7] - prevData[7]) > 0) ? (resourceRequestBitmap | (1 << 7)) : resourceRequestBitmap;
	/* Flash Write Issue */
	resourceRequestBitmap = ((curData[8] - prevData[8]) > 0) ? (resourceRequestBitmap | (1 << 8)) : resourceRequestBitmap;
	/* Flash Write Sector */
	resourceRequestBitmap = ((curData[9] - prevData[9]) > 0) ? (resourceRequestBitmap | (1 << 9)) : resourceRequestBitmap;
	/* SDcard Read Issue */
	resourceRequestBitmap = ((curData[10] - prevData[10]) > 0) ? (resourceRequestBitmap | (1 << 10)) : resourceRequestBitmap;
	/* SDcard Read Sector */
	resourceRequestBitmap = ((curData[11] - prevData[11]) > 0) ? (resourceRequestBitmap | (1 << 11)) : resourceRequestBitmap;
	/* SDcard Write Issue */
	resourceRequestBitmap = ((curData[12] - prevData[12]) > 0) ? (resourceRequestBitmap | (1 << 12)) : resourceRequestBitmap;
	/* SDcard Write Sector */
	resourceRequestBitmap = ((curData[13] - prevData[13]) > 0) ? (resourceRequestBitmap | (1 << 13)) : resourceRequestBitmap;
	/* touch interrupt */
	resourceRequestBitmap = ((curData[14] - prevData[14]) > 0) ? (resourceRequestBitmap | (1 << 14)) : resourceRequestBitmap;
	/* timer interrupt */
	resourceRequestBitmap = ((curData[15] - prevData[15]) > 0) ? (resourceRequestBitmap | (1 << 15)) : resourceRequestBitmap;
	/* 3G Receive */
	resourceRequestBitmap = ((curData[16] - prevData[16]) > 0) ? (resourceRequestBitmap | (1 << 16)) : resourceRequestBitmap;
	/* 3G Send */
	resourceRequestBitmap = ((curData[17] - prevData[17]) > 0) ? (resourceRequestBitmap | (1 << 17)) : resourceRequestBitmap;
	/* Wifi Receive */
	resourceRequestBitmap = ((curData[18] - prevData[18]) > 0) ? (resourceRequestBitmap | (1 << 18)) : resourceRequestBitmap;
	/* Wifi Send */
	resourceRequestBitmap = ((curData[19] - prevData[19]) > 0) ? (resourceRequestBitmap | (1 << 19)) : resourceRequestBitmap;

	if( list->size == 0 ) 
	{
		list->current = insertState( list, resourceRequestBitmap, (prevData[0] + prevData[1]), prevData[0] );
	}
	else if( list->current->resourceRequestBitmap != resourceRequestBitmap ) /* jump to next state */
	{
		State *tmp;
		tmp = hasSameBitmap( list, resourceRequestBitmap );

		int targetFreqLevel;
		targetFreqLevel = MAX_CPU_LEVEL - 1;
		if( tmp == NULL ) 
		{
			tmp = insertState( list, resourceRequestBitmap, (curData[0] + curData[1]), curData[0] );
		}
		else 
		{
			(tmp->count)++;	
			tmp->accuWorkload = curData[0];
			tmp->accuDuration = curData[0] + curData[1];

			/* Prediction START */	

			double predict = 0;
			/* if no data no predict */
			if( tmp->pathSize != 0 ) 
				//if( list->current->pathSize != 0 && list->current->pathAccuCount > (MAX_OF_CT_RECENT+MAX_OF_CT_PAST-1) )
			{   
				Path *p; 
				p = tmp->head;
				while(p != NULL)
				{   
					predict += ((double)p->count / tmp->pathAccuCount) * getValueFromCorTable(p->T);
					p = p->next;
				}   
				double targetFreq = predict * freq_table[MAX_CPU_LEVEL - 1] / 100;
				//printf(" predict = %.2f, target = %.2f\n", predict, targetFreq);

				if( targetFreq < freq_table[0] ) targetFreqLevel = 0;
				else if( targetFreq >= freq_table[MAX_CPU_LEVEL - 1] ) targetFreqLevel = MAX_CPU_LEVEL - 1;
				else
				{
					for( targetFreqLevel = 1 ; targetFreqLevel < MAX_CPU_LEVEL ; targetFreqLevel++ )
					{
						if( targetFreq >= freq_table[targetFreqLevel - 1] && targetFreq < freq_table[targetFreqLevel]  ) break;
					}
				}
			}   		
			/* Prediction END*/
		}

		if( curFreqLevel != targetFreqLevel  ) 
		{
			curFreqLevel = targetFreqLevel;
			//printf("level = %d, freq = %d\n", curFreqLevel, freq_table[curFreqLevel]);
			setCPULevel(curFreqLevel);
		}

		updateState( list->current, curData[0], curData[1], tmp->ID);
		list->current = tmp;
	}

	return ;
}

State* hasSameBitmap( List *list, int bitmap )
{
	State *tmp;
	tmp = list->head;
	do
	{	
		if( tmp->resourceRequestBitmap == bitmap ) return tmp;
		tmp = tmp->next;
	}while( tmp != NULL );
	return tmp;
}

void showBitmap(int data, int digit)
{
	int i;
	for( i = (digit-1) ; i >=0 ; i-- ) printf("%d", (data & (1 << i)) >> i);
	printf("\n");
}

Path *hasSamePath( Path *head, int ID)
{
	Path *tmp;
	tmp = head;
	while( tmp != NULL )
	{
		if(tmp->destID == ID) return tmp;
		else tmp = tmp->next;
	}
	return NULL;
}

CorTable * initCorTable(void)
{
	CorTable *node;
	node = (CorTable *)malloc(sizeof(CorTable));
	node->indexOfRecent = 0;
	node->indexOfPast = 0;
	node->countOfRecent = 0;
	node->countOfPast = 0;
	node->meanOfRecent = 0;
	node->meanOfPast = 0;

	return node;
}

double getValueFromCorTable(CorTable *T)
{
	double alpha, beta;
	alpha = 1;
	beta = 0;
	if( T->countOfPast != 0 ) return alpha * T->meanOfRecent + beta * T->meanOfPast;
	else return T->meanOfRecent;
}

void updateCorTable(CorTable *T, double utilization)
{
	/* normalize to MAX CPU frequency */
	utilization = utilization * freq_table[curFreqLevel] / freq_table[MAX_CPU_LEVEL-1];

	if( T->countOfRecent == MAX_OF_CT_RECENT )
	{
		T->meanOfRecent += ((double)(utilization - T->utilOfRecent[T->indexOfRecent]) / T->countOfRecent);
		if( T->countOfPast == MAX_OF_CT_PAST )
		{
			T->meanOfPast += ((double)(T->utilOfRecent[T->indexOfRecent] - T->utilOfPast[T->indexOfPast]) / T->countOfPast);
		}
		else 
		{
			T->countOfPast++;
			T->meanOfPast = ((double)(T->meanOfPast * (T->countOfPast - 1)) / T->countOfPast) + ((double)T->utilOfRecent[T->indexOfRecent] / T->countOfPast);
		}
		T->utilOfPast[T->indexOfPast] = T->utilOfRecent[T->indexOfRecent];
		T->indexOfPast = (T->indexOfPast + 1) % MAX_OF_CT_PAST;

	}
	else 
	{
		T->countOfRecent++;
		T->meanOfRecent = ((double)(T->meanOfRecent * (T->countOfRecent - 1)) / T->countOfRecent) + ((double)utilization / T->countOfRecent);
	}
	T->utilOfRecent[T->indexOfRecent] = utilization;
	T->indexOfRecent = (T->indexOfRecent + 1) % MAX_OF_CT_RECENT;

	return ;
}

void updateState( State *cur, int busy, int idle, int nextStateID )
{
	int workload, duration;
	workload =  busy - cur->accuWorkload;
	duration =  busy + idle - cur->accuDuration;
	cur->meanWorkload = ((double)(cur->meanWorkload * (cur->count - 1)) / cur->count) + ((double)workload / cur->count);
	cur->meanDuration = ((double)(cur->meanDuration * (cur->count - 1)) / cur->count) + ((double)duration / cur->count);
	cur->accuWorkload = cur->accuDuration = 0;
	if( nextStateID != -1)
	{
		Path *tmp;
		tmp = hasSamePath(cur->head, nextStateID);
		if( tmp == NULL ) /* create new path */
		{
			(cur->pathSize)++;
			/* init a path */
			tmp = (Path *)malloc(sizeof(Path));
			tmp->destID = nextStateID;
			tmp->count = 0;
			tmp->next = NULL;
			tmp->T = initCorTable();
			if(cur->head == NULL) cur->head = tmp;
			else cur->rear->next = tmp;
			cur->rear = tmp;
		}
		updateCorTable(tmp->T, (double)workload * 100 / duration);

		(tmp->count)++;                        
		if( cur->pathAccuCount == PATH_QUEUE_SIZE )                         
		{                                   
			Path *tmp1;                                
			tmp1 = hasSamePath(cur->head, cur->pathQueueID[cur->pathQueueTop]);                                
			(tmp1->count)--;                        
		}                        
		else cur->pathAccuCount++;                        
		cur->pathQueueID[cur->pathQueueTop] = tmp->destID;
		cur->pathQueueTop = (cur->pathQueueTop + 1) % PATH_QUEUE_SIZE;
	}
}

double computeSTD(int *data, int size)
{
	int i;
	double mean, sum;
	sum = 0;
	for( i = 0 ; i < size ; i++ ) sum += data[i];
	mean = sum / size;

	sum = 0;
	for( i = 0 ; i < size ; i++ ) sum += ((data[i] - mean) * (data[i] - mean));
	sum /= size;
	return sqrt(sum);
}

double computeMEAN(int *data, int size)
{
	int i;
	double mean;
	mean = 0;
	for( i = 0 ; i < size ; i++ ) mean += data[i];
	return (mean /= size);
}

void stat() // called by timer
{

	char buff[128+1];
	unsigned long long tmp;
	int i;

	/*  CPU  */
	FILE *fp;
	fp = fopen("/proc/stat","r");
	while( fgets(buff, 128, fp) )
	{
		if( strstr(buff, "cpu ") )
		{
			sscanf(buff, "cpu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu", &CPUInfo[0], &CPUInfo[1], &CPUInfo[2], &CPUInfo[3], &CPUInfo[4], &CPUInfo[5], &CPUInfo[6], &CPUInfo[7], &CPUInfo[8], &CPUInfo[9]) ; // time(unit: jiffies) spent of all cpus for: user nice system idle iowait irq softirq stead guest guest_nice
		}
		else if( strstr(buff, "ctxt") )
		{
			sscanf(buff, "ctxt %llu", &ctxt); // times of context switch of all cpus
		}
		else if( strstr(buff, "procs_running") )
		{
			sscanf(buff, "procs_running %llu", &processR); // # of processes running
		}
		else if( strstr(buff, "softirq") )
		{
			sscanf(buff, "softirq %llu", &softirq); // sum_softirq
			break;
		}

	}
	fclose(fp);

	/* load avg  */
	float loadavg1, loadavg5, loadavg15;
	fp = fopen("/proc/loadavg","r");
	fscanf(fp, "%f %f %f", &loadavg1, &loadavg5, &loadavg15);
	fclose(fp);

	/* RAM */
	fp = fopen("/proc/meminfo","r");
	while( fgets(buff, 128, fp) )
	{
		if( strstr(buff, "MemFree") )   /* memory for program (KB) */
		{
			sscanf(buff, "MemFree: %llu", &memFree);
		}
		else if( strstr(buff, "Cached") ) /* memory for cache (KB) */
		{
			sscanf(buff, "Cached: %llu", &memCache);
			break;
		}
	}
	fclose(fp);

	/* FlASH and SDcard */
	fp = fopen("/proc/diskstats","r");
	while( fgets(buff, 128, fp) )
	{
		if( strstr(buff, "mmcblk0p9") )        /* system */
		{
			sscanf(buff, " 179       9 mmcblk0p9 %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu", &systemFlash[0], &systemFlash[1], &systemFlash[2], &systemFlash[3], &systemFlash[4], &systemFlash[5], &systemFlash[6], &systemFlash[7], &systemFlash[8], &systemFlash[9], &systemFlash[10]);
		}
		else if( strstr(buff, "mmcblk0p12") )   /* data */
		{
			sscanf(buff, " 179      12 mmcblk0p12 %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu", &dataFlash[0], &dataFlash[1], &dataFlash[2], &dataFlash[3], &dataFlash[4], &dataFlash[5], &dataFlash[6], &dataFlash[7], &dataFlash[8], &dataFlash[9], &dataFlash[10]);
		}
		else if( strstr(buff, "mmcblk0p8") )   /* cache */
		{
			sscanf(buff, " 179       8 mmcblk0p8 %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu", &cacheFlash[0], &cacheFlash[1], &cacheFlash[2], &cacheFlash[3], &cacheFlash[4], &cacheFlash[5], &cacheFlash[6], &cacheFlash[7], &cacheFlash[8], &cacheFlash[9], &cacheFlash[10]);
		}
		else if( strstr(buff, "mmcblk0p3") )   /* EFS */
		{
			sscanf(buff, " 179       3 mmcblk0p3 %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu", &devlogFlash[0], &devlogFlash[1], &devlogFlash[2], &devlogFlash[3], &devlogFlash[4], &devlogFlash[5], &devlogFlash[6], &devlogFlash[7], &devlogFlash[8], &devlogFlash[9], &devlogFlash[10]);
		}
		else if( strstr(buff, "mmcblk0p4") )   /* PARAM */
		{
			sscanf(buff, "  179       4 mmcblk0p4 %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu", &paramFlash[0], &paramFlash[1], &paramFlash[2], &paramFlash[3], &paramFlash[4], &paramFlash[5], &paramFlash[6], &paramFlash[7], &paramFlash[8], &paramFlash[9], &paramFlash[10]);
		}
		else if( strstr(buff, "mmcblk0p7") )   /* RADIO */
		{
			sscanf(buff, " 179       7 mmcblk0p7 %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu", &radioFlash[0], &radioFlash[1], &radioFlash[2], &radioFlash[3], &radioFlash[4], &radioFlash[5], &radioFlash[6], &radioFlash[7], &radioFlash[8], &radioFlash[9], &radioFlash[10]);
		}
		else if( strstr(buff, "mmcblk1p1") ) /* External sdcard */
		{
			sscanf(buff, " 179      17 mmcblk1p1 %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu", &SDcard[0], &SDcard[1], &SDcard[2], &SDcard[3], &SDcard[4], &SDcard[5], &SDcard[6], &SDcard[7], &SDcard[8], &SDcard[9], &SDcard[10]);
			break;
		}
	}
	fclose(fp);

	/* interrupt */
	fp = fopen("/proc/interrupts","r");
	while( fgets(buff, 128, fp) )
	{
		/*if( strstr(buff, "gp_timer") )
		{
			sscanf(buff, "  1: %llu", &timerIntr);
		}*/
		if( strstr(buff, "melfas-ts") )
		{
			sscanf(buff, "387:	%llu", &touchIntr);
		}
		else if( strstr(buff, "Local timer interrupts") )
		{
			sscanf(buff, "LOC:     %llu %llu %llu %llu", &localTimerIntr[0], &localTimerIntr[1], &localTimerIntr[2], &localTimerIntr[3]);
			break;
		}

	}
	fclose(fp);

	/* network */
	fp = fopen("/proc/net/dev","r");
	Net3GR = Net3GS = NetWifiR = NetWifiS = 0;
	while( fgets(buff, 128, fp) )
	{
		if( strstr(buff, "rmnet0") )
		{
			sscanf(buff, "rmnet0: %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu", &Net3GR, &tmp, &tmp, &tmp, &tmp, &tmp, &tmp, &tmp, &Net3GS, &tmp, &tmp, &tmp, &tmp, &tmp, &tmp, &tmp);
		}
		else if( strstr(buff, "wlan0") )
		{
			sscanf(buff, " wlan0: %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu", &NetWifiR, &tmp, &tmp, &tmp, &tmp, &tmp, &tmp, &tmp, &NetWifiS, &tmp, &tmp, &tmp, &tmp, &tmp, &tmp, &tmp);
			break;
		}
	}
	fclose(fp);

	/* Read Trace into data[20] */
	curData[0] = (double)(CPUInfo[0]+CPUInfo[1]+CPUInfo[2]+CPUInfo[4]+CPUInfo[5]+CPUInfo[6]+CPUInfo[7]);
	curData[1] = (double)CPUInfo[3];
	curData[2] = processR;
	curData[3] = softirq;
	curData[4] = memFree;
	curData[5] = memCache;
	curData[6] = (systemFlash[0]+dataFlash[0]+cacheFlash[0]+devlogFlash[0]+paramFlash[0]+radioFlash[0]);
	curData[7] = (systemFlash[2]+dataFlash[2]+cacheFlash[2]+devlogFlash[2]+paramFlash[2]+radioFlash[2]);
	curData[8] = (systemFlash[4]+dataFlash[4]+cacheFlash[4]+devlogFlash[4]+paramFlash[4]+radioFlash[4]);
	curData[9] = (systemFlash[6]+dataFlash[6]+cacheFlash[6]+devlogFlash[6]+paramFlash[6]+radioFlash[6]);
	curData[10] = SDcard[0];
	curData[11] = SDcard[2];
	curData[12] = SDcard[4];
	curData[13] = SDcard[6];
	curData[14] = touchIntr;
	curData[15] = localTimerIntr[0]+localTimerIntr[1]+localTimerIntr[2]+localTimerIntr[3];
	curData[16] = Net3GR;
	curData[17] = Net3GS;
	curData[18] = NetWifiR;
	curData[19] = NetWifiR;

#ifdef DEBUG
	for( i = 0 ; i < 20 ; i++ ) printf("%.5f\t", curData[i]);
	printf("\n");
#endif

	/* Build State Transition Diagram */
	if( prevData[0] != 0 )
	{
		checkNewState( &stateList, curData, prevData, RESOURCE_NUMBER );
	}

	/* Save last data */
	for( i = 0 ; i < 20 ; i++ ) prevData[i] = curData[i];

	if( stopFlag == 1 )
	{
		tick.it_value.tv_sec = 0 ;
		tick.it_value.tv_usec = 0 ;
		tick.it_interval.tv_sec = 0 ;
		tick.it_interval.tv_usec = 0 ;
		res = setitimer(ITIMER_REAL, &tick, NULL);

		printf("End!!!\n") ;
		stopFlag = 2;
		exit(0);
	}

}

void setCPULevel(int level)
{
	FILE *fp;
	fp = fopen("/sys/devices/system/cpu/cpu0/cpufreq/scaling_setspeed","w");
	if(fp)
	{   
		fprintf(fp,"%d\n",freq_table[level]);
	}   
	else printf("setCPULevel failed!!!");
	fclose(fp);
}

