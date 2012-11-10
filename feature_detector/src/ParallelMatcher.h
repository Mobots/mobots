#include <semaphore.h>
#include <list>
#include <pthread.h>

#include "feature_detector/FeaturesFinder.h"
#include "feature_detector/FeaturesMatcher.h"

#pragma once

class ResultListener{
	public:
		/**
		 * 
		 */
		void onResult(MatchResult result, int id1, int id2);
		/**
		 * dont block in this method
		 */
		void onLowWatermark();
		/**
		 * dont block in this method
		 */
		void onHighWatermark();
};

class ParallelMatcher{
public:
	ParallelMatcher(ResultListener* listener, int threadCount, int lowWatermark, int highWatermark);
	~ParallelMatcher();
	void enqueue(const FeatureSet* set1, const FeatureSet* set2, int id1, int id2);

private:
	
	typedef struct{
		const FeatureSet* set1;
		const FeatureSet* set2;
		int id1;
		int id2;
	}entry;
	
	typedef struct{
		MatchResult matchResult;
		int id1;
		int id2;
	}matchEntry;
	
	typedef struct{
		ParallelMatcher* parallelMatcher;
		int method;
	}callable;
	
	void spinMethod();
	void resultDeliverer();
	const entry dequeueFeaturePair();
	
	
	bool ok;
	ResultListener* listener;
	int threadCount;
	int lowWatermark;
	int highWatermark;
	pthread_t* threads;
	pthread_t resultThread;
	sem_t counterSem, resultCounterSem;
	sem_t queue_sem, result_queue_sem;
	callable callable;
	std::deque<entry> queue;
	std::deque<matchEntry> resultQueue;
};
