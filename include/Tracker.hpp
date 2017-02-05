#ifndef TRACKER_H
#define TRACKER_H

#include "BaseBayesianTracker.hpp"

template <class Obj, class PosEstim> class Tracker : public BaseBayesianTracker<Obj>
{
public:

	std::shared_ptr<PosEstim> positionEstimator;

	Tracker(std::shared_ptr<Obj> objectPTR, std::shared_ptr<PosEstim> positionEstimator) : positionEstimator(positionEstimator)
	{
		this->objectPTR = objectPTR;
		obsSize = objectPTR->getObservableStates().size();
		this->trackerType = objectPTR->getObjectType();
	};

	~Tracker() {}

	void preProcessingComputations()
	{
	};

	void predict(VectorXd &controlVect)
	{
		positionEstimator->predict(controlVect);
		VectorXd state = positionEstimator->getStatePred();
		MatrixXd stateCov = positionEstimator->getCovPred();

        	this->objectPTR->setState(state.head(obsSize));
        	this->objectPTR->setCovariance(stateCov.block(0, 0, obsSize, obsSize));

	};

	void postPredictComputations() {};

	void update(Detection<Obj> &det)
	{

		VectorXd detPosition = det.getObjPTR()->getObservableStates();


		positionEstimator->update(detPosition);

		VectorXd state = positionEstimator->getStatePost();
		MatrixXd stateCov = positionEstimator->getCovPost();

        	this->objectPTR->setState(state.head(obsSize));
        	this->objectPTR->setCovariance(stateCov.block(0, 0, obsSize, obsSize));
	};

	void postUpdateComputations() {};

	shared_ptr<BaseTracker<Obj>> clone()
	{
		shared_ptr<PosEstim> estimClone = static_pointer_cast<PosEstim>(this->positionEstimator->clone());
		shared_ptr<Obj> objClone = static_pointer_cast<Obj>(this->objectPTR->clone());

		return shared_ptr<BaseTracker<Obj>>(new Tracker<Obj, PosEstim>(objClone, estimClone));
	}
	
private:
	int obsSize;
};



#endif // !TRACKER_H
