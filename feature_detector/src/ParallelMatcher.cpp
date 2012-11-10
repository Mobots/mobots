#include "ParallelMatcher.h"
#include "feature_detector/FeaturesMatcher.h"
#include <opencv2/core/core.hpp>
#include <iostream>

using namespace std;

void* threadHelper(void* data){
	callable* callable = (callable*) data;
	switch(callable->method){
		case 1:
			callable->parallelMatcher->spinMethod();
			break;
		case 2:
			callable->parallelMatcher->resultDeliverer();
			break;
	}
	return 0;
}

ParallelMatcher::ParallelMatcher(ResultListener* listener, int threadCount, int lowWatermark, int highWatermark)
:threadCount(threadCount), listener(listener){
	if(sem_init(&counterSem, 0, 0) < 0 || sem_init(&queue_sem, 0, 0) < 0)
		cerr << __PRETTY_FUNCTION__ << " error: semaphore" << endl;
	if(sem_init(&resultCounterSem, 0, 0) < 0 || sem_init(&result_queue_sem, 0, 0) < 0)
		cerr << __PRETTY_FUNCTION__ << " error: semaphore" << endl;
	
	callable.parallelMatcher = this;
	callable.method = 1;
	for(int i = 0; i < threadCount; i++){
		pthread_create(threads[i], 0, spinMethod, &callable);
	}
	callable.method = 2;
	pthread_create(&resultThread, 0, resultDeliverer, &callable);
}

void ParallelMatcher::spinMethod(){
	cv::Ptr<FeaturesMatcher> matcher = FeaturesMatcher::getDefault();
	while(ok){
		entry entry = dequeueFeaturePair();
		matchEntry matchEntry;
		bool result = matcher->match(*entry.set1, *entry.set2, matchEntry.matchResult);
		if(!result){
			matchEntry.id1 = -1;
			matchEntry.id2 = -1;
		}else{
			matchEntry.id1 = entry.id1;
			matchEntry.id2 = entry.id2;
		}
		
		sem_wait(&result_queue_sem);
		resultQueue.push_back(entry);
		sem_post(&result_queue_sem);
		
		sem_post(&resultCounterSem);
		
	}
	return 0;
}

const ParallelMatcher::entry ParallelMatcher::dequeueFeaturePair(){
	sem_wait(&counterSem);
	
	sem_wait(&queue_sem);
	entry entry = queue.front();
	queue.pop_front();
	sem_post(&queue_sem);
	
	return entry;
}

void ParallelMatcher::enqueue(const FeatureSet* set1, const FeatureSet* set2, int id1, int id2){
	
	entry entry;
	entry.set1 = set1;
	entry.set2 = set2;
	entry.id1 = id1;
	entry.id2 = id2;
	
	sem_wait(&queue_sem);
	queue.push_back(entry);
	sem_post(&queue_sem);
	
	sem_post(&counterSem);
	
	if(queue.size() > highWatermark)
		listener->onHighWatermark();
		
}

void ParallelMatcher::resultDeliverer(){
	while(ok){
		
		sem_wait(&resultCounterSem);
		
		sem_wait(&result_queue_sem);
		matchEntry entry = resultQueue.front();
		resultQueue.pop_front();
		sem_post(&result_queue_sem);
		
		listener->onResult(entry.matchResult, entry.id1, entry.id2);
		if(resultQueue.size() < lowWatermark)
			listener->onLowWatermark();
	}
}

ParallelMatcher::~ParallelMatcher(){ //meh todo
	ok = false;
	sem_destroy(&queue_sem);
	sem_destroy(&counterSem);
}
