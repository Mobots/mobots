#include "feature_detector/ParallelMatcher.h"
#include "feature_detector/FeaturesMatcher.h"
#include <opencv2/core/core.hpp>
#include <iostream>

using namespace std;


ParallelMatcher::ParallelMatcher(ParallelMatcher::ResultListener* listener, int threadCount, int lowWatermark, int highWatermark)
:listener(listener), threadCount(threadCount), lowWatermark(lowWatermark), highWatermark(highWatermark){
	if(sem_init(&counterSem, 0, 0) < 0 || sem_init(&queue_sem, 0, 1) < 0)
	 cerr << __PRETTY_FUNCTION__ << " error: semaphore" << endl;
	if(sem_init(&resultCounterSem, 0, 0) < 0 || sem_init(&result_queue_sem, 0, 1) < 0)
	 cerr << __PRETTY_FUNCTION__ << " error: semaphore" << endl;
  if(sem_init(&result_deliver_sem, 0, 1) < 0)
	 cerr << __PRETTY_FUNCTION__ << " error: semaphore" << endl;
  for(int i = 0; i < threadCount; i++){
	 workers.push_back(new Worker(this));
  }
  deliverThread = boost::thread(&ParallelMatcher::deliverThread, this);
}

void ParallelMatcher::enqueueResult(const MatchResultEntry& entry){
  sem_wait(&result_queue_sem);
  resultQueue.push_back(entry);
  sem_post(&result_queue_sem);
  
  sem_post(&resultCounterSem);
}

const ParallelMatcher::FeaturesEntry ParallelMatcher::dequeueFeaturePair(){
	sem_wait(&counterSem);
	
	sem_wait(&queue_sem);
	FeaturesEntry entry = queue.front();
	queue.pop_front();
	cout << "dequeing " << entry.id1 << "-" << entry.id2 << endl;
	sem_post(&queue_sem);
	
	return entry;
}

void ParallelMatcher::enqueue(const FeatureSet* set1, const FeatureSet* set2, int id1, int id2){
	
	FeaturesEntry entry;
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
	
	cout << "queue size " << queue.size() << "got " << set1 << " and " << set2 << endl;
		
}

void ParallelMatcher::resultDeliverer(){
  cout << "deliverer" << endl;
	while(ok){
		
		sem_wait(&resultCounterSem);
		
		sem_wait(&result_queue_sem);
		MatchResultEntry entry = resultQueue.front();
		resultQueue.pop_front();
		sem_post(&result_queue_sem);
		
		sem_wait(&result_deliver_sem);
		cout << "sending result" << entry.id1 << "-" << entry.id2 << " is " << entry.matchable << endl;
		listener->onResult(entry.matchResult, entry.matchable, entry.id1, entry.id2);
		sem_post(&result_deliver_sem);
		
		if(resultQueue.size() < lowWatermark)
			listener->onLowWatermark();
	}
}

ParallelMatcher::~ParallelMatcher(){ //meh todo
	ok = false;
	sem_destroy(&queue_sem);
	sem_destroy(&counterSem);
}

ParallelMatcher::Worker::Worker(ParallelMatcher* parent):parent(parent) {
  me = boost::thread(&ParallelMatcher::Worker::spinMethod, this);
}

void ParallelMatcher::Worker::spinMethod(){
  cout << "spinMethod" << endl;
	cv::Ptr<FeaturesMatcher> matcher = FeaturesMatcher::getDefault();
	while(parent->ok){
		FeaturesEntry entry = parent->dequeueFeaturePair();
		MatchResultEntry matchEntry;
		cout << "matching " << entry.set1 << " and " << entry.set2 << endl;
		matchEntry.matchable = matcher->match(*entry.set1, *entry.set2, matchEntry.matchResult);
		matchEntry.id1 = entry.id1;
		matchEntry.id2 = entry.id2;
		cout << "trying to enqueue result" << matchEntry.id1 << "-" << matchEntry.id2 << " is " << matchEntry.matchable << endl;
		parent->enqueueResult(matchEntry);
	}
}
