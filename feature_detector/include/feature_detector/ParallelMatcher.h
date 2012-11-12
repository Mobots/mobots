#include <semaphore.h>
#include <list>
#include <boost/thread.hpp> 

#include "feature_detector/FeaturesFinder.h"
#include "feature_detector/FeaturesMatcher.h"

#pragma once

class ParallelMatcher{
public:
  
  class ResultListener{
  public:
	 /**
		* 
		*/
	 virtual void onResult(MatchResult result, bool matachable, int id1, int id2) = 0;
	 /**
		* dont block in this method
		*/
	 virtual void onLowWatermark() = 0;
	 /**
		* dont block in this method
		*/
	 virtual void onHighWatermark() = 0;
};
  
	ParallelMatcher(ParallelMatcher::ResultListener* listener, int threadCount, int lowWatermark, int highWatermark);
	~ParallelMatcher();
	void enqueue(const FeatureSet* set1, const FeatureSet* set2, int id1, int id2);
    ParallelMatcher* parent;

private:
  
  class Worker{
  public:
	 Worker(ParallelMatcher* parent);
  private:
	 void spinMethod();
	 ParallelMatcher* parent;
	 boost::thread me;
  };
	
	typedef struct{
		const FeatureSet* set1;
		const FeatureSet* set2;
		int id1;
		int id2;
	}FeaturesEntry;
	
	typedef struct{
		MatchResult matchResult;
		int id1;
		int id2;
		bool matchable;
	}MatchResultEntry;
	
	void resultDeliverer();
	const FeaturesEntry dequeueFeaturePair();
	void enqueueResult(const MatchResultEntry&);
	
	bool ok;
	ResultListener* listener;
	int threadCount;
	int lowWatermark;
	int highWatermark;
	std::vector<Worker*> workers;
	boost::thread deliverThread;
	sem_t counterSem, resultCounterSem;
	sem_t queue_sem, result_queue_sem;
	sem_t result_deliver_sem;
	std::deque<FeaturesEntry> queue;
	std::deque<MatchResultEntry> resultQueue;
};
