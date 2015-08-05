#pragma once
#include "NeuralNet.h"

using namespace std;

double NeuralNet::randAddFanIn(double fan_in){
	// Adds random amount mutationRate% of the time, amount based on fan_in and mutstd
	if (double(rand())/double(RAND_MAX)>mutationRate){
		return 0.0;
	} else {
		// FOR MUTATION
		std::default_random_engine generator;
		generator.seed(time(NULL));
		std::normal_distribution<double> distribution(0.0,mutStd);
		return distribution(generator);
	}
}

double NeuralNet::randSetFanIn(double fan_in){
	// For initialization of the neural net weights
		double rand_neg1to1 = ((double(rand())/double(RAND_MAX))*2.0-1.0)*0.1;
		double scale_factor = 100.0;
		return scale_factor*rand_neg1to1/sqrt(fan_in);
}

void NeuralNet::mutate(){
	for (int i=0; i<Wbar.size(); i++){
		for (int j=0; j<Wbar[i].size(); j++){
#pragma parallel omp for
			for (int k=0; k<Wbar[i][j].size(); k++){
				double fan_in = double(Wbar[i].size());
				Wbar[i][j][k] += randAddFanIn(fan_in);
			}
		}
	}
}

void NeuralNet::setRandomWeights(){
	Wbar = matrix3d(connections());
	W = matrix3d(connections());
	for (int connection=0; connection<connections(); connection++){ // number of layers
		int above = connection;
		int below = connection+1;

		// Populate Wbar with small random weights, including bias
		Wbar[connection] = (matrix2d(nodes_[above]+1));
		for (int i=0; i<nodes_[above]+1; i++){ // above+1 include bias;
			Wbar[connection][i] = matrix1d(nodes_[below]); // reserve memory for the connections below
			for (int j=0; j<nodes_[below]; j++){
				double fan_in = nodes_[above]+1.0;
				Wbar[connection][i][j] = randSetFanIn(fan_in);
			}
		}

		W[connection] = Wbar[connection];
		W[connection].pop_back(); // remove extra bias weights
	};
}

NeuralNet::NeuralNet(int nInputs, int nHidden, int nOutputs, double gamma):nodes_(vector<int>(3)),gamma_(gamma),
	evaluation(0),mutationRate(0.5),mutStd(1.0){
	nodes_[0] = nInputs;
	nodes_[1] = nHidden;
	nodes_[2] = nOutputs;

	setRandomWeights();
	setMatrixMultiplicationStorage();
}

void NeuralNet::setMatrixMultiplicationStorage(){
	// Allocates space for the matrix multiplication storage container, based on current Wbar/connections()
	matrix_multiplication_storage = matrix2d(connections());
	for (int connection=0; connection<connections(); connection++){
		matrix_multiplication_storage[connection] = matrix1d(Wbar[connection][0].size(),0.0);
		if (connection+1!=connections()){ // if not the output layer
			matrix_multiplication_storage[connection].push_back(1.0);
		}
	}
}

void NeuralNet::addInputs(int nToAdd){
	nodes_[0] += nToAdd;

	for (int i=0; i<nToAdd; i++){ // add new connections leading to each of the lower nodes
		Wbar[0].push_back(matrix1d(nodes_[1],0.0)); // adding another connection, each new one leads to nodes below
		/*for (int j=0; j<nodes_[1]; j++){
			double fan_in = nodes_[0]+1.0;
			double rand_neg1to1 = (double(rand())/double(RAND_MAX))*2.0-1.0;
			Wbar[0].back()[j]=rand_neg1to1/sqrt(fan_in);
			Wbar[0].back()[j]=0.0;
		}*/
	}
	W[0] = Wbar[0];
	W[0].pop_back();

	setMatrixMultiplicationStorage();
}

NeuralNet::NeuralNet(vector<int> &nodes, double gamma):evaluation(0.0),nodes_(nodes),gamma_(gamma){
	setRandomWeights();
	setMatrixMultiplicationStorage();
}

void NeuralNet::train(matrix2d &observations, matrix2d &T, double epsilon, int iterations){
	double err = 2*epsilon+1.0; // just ensure it's bigger always to begin...
	int step = 0;
	if (iterations==0){
		while(err>=epsilon){
			matrix1d errs;
			for (int i=0; i<observations.size();  i++){
				errs.push_back(backProp(observations[i],T[i]));
			}
			err = sum(errs);
			printf("Err=%f\n",err);
		}
	} else {
		while(err>=epsilon && iterations>=step){
			matrix1d errs;
			for (int i=0; i<observations.size();  i++){
				errs.push_back(backProp(observations[i],T[i]));
			}
			err = sum(errs);
			printf("Err=%f\n",err);
			step++;
		}
	}
}

matrix1d NeuralNet::predictBinary(matrix1d observations){
	for (int connection=0; connection<connections(); connection++){
		//printf("size(Wbar[connection])=%i",Wbar[connection].size());
		observations.push_back(1.0); // add 1 for bias
		//printf("PREDICT: Asize=%i,Bsize=%i",observations.size(),Wbar[connection].size());
		observations = matrixMultiply(observations,Wbar[connection]);
		sigmoid(observations); // Compute outputs
	}
	return observations;
}

matrix1d NeuralNet::predictContinuous(matrix1d observations){

	observations.push_back(1.0);
	matrixMultiply(observations,Wbar[0],matrix_multiplication_storage[0]);
	sigmoid(matrix_multiplication_storage[0]);

	for (int connection=1; connection<connections(); connection++){
		matrix_multiplication_storage[connection-1].back() = 1.0; // static size allocation... last element is set to 1.0, bias (may not need to?)
		matrixMultiply(matrix_multiplication_storage[connection-1],Wbar[connection],matrix_multiplication_storage[connection]);
		sigmoid(matrix_multiplication_storage[connection]);
	}

	return matrix_multiplication_storage.back();

}

matrix2d NeuralNet::batchPredictBinary(matrix2d &observations){
	matrix2d out;
	for (int i=0; i<observations.size(); i++){
		out.push_back(predictBinary(observations[i]));
	}
	return out;
}

matrix2d NeuralNet::batchPredictContinuous(matrix2d &observations){
	matrix2d out;
	for (int i=0; i<observations.size(); i++){
		out.push_back(predictContinuous(observations[i]));
	}
	return out;
}

double NeuralNet::sum(matrix1d &myVector){
	double mySum = 0.0;
	for (int i=0; i<myVector.size(); i++){
		mySum+=myVector[i];
	}
	return mySum;
}

double NeuralNet::SSE(matrix1d &myVector){
	double err = 0.0;
	for (int i=0; i<myVector.size(); i++){
		err += myVector[i]*myVector[i];
	}
	return err;
}

int NeuralNet::connections(){
	return nodes_.size()-1;
}

double NeuralNet::backProp(matrix1d &observations, matrix1d &t){
	// 'observations' is the input vector, 't' is the 'target vector'
	// returns the SSE for the output vector

	matrix2d Ohat; // outputs with bias
	matrix3d D; // stored derivatives
	// Go through network "feed forward" computation

	feedForward(observations,Ohat,D);

	matrix1d e(Ohat.back().size()-1,0.0); // "stored derivatives of the quadratic deviations"
	for (int i=0; i<Ohat.back().size()-1; i++){
		e[i] = (Ohat.back()[i]-t[i]);
	}

	// Hidden/output layer delta calcs
	matrix2d delta;
	for (int i=0; i<connections(); i++){
		delta.push_back(matrix1d());
	}
	delta.back() = matrixMultiply(D.back(),e); // output layer delta

	for (int connection=connections()-2; connection>=0; connection--){ // back propagation
		matrix2d temp = matrixMultiply(D[connection],W[connection+1]) ;
		delta[connection]=matrixMultiply(temp,delta[connection+1]);
	}

	// Corrections to weights
	for (int connection=0; connection<connections(); connection++){
		matrix2d DeltaWbarT = matrixMultiply(delta[connection],Ohat[connection]);
		for (int i=0; i<Wbar[connection].size(); i++){
			for (int j=0; j<Wbar[connection][i].size(); j++){
				Wbar[connection][i][j] -= gamma_*DeltaWbarT[j][i]; // ji because it's transpose :)
			}
		}

		W[connection] = Wbar[connection];
		W[connection].pop_back();
	}

	// Calculate SS
	return SSE(e);
}

void NeuralNet::feedForward(matrix1d &observations, matrix2d &Ohat, matrix3d &D){
	Ohat.push_back(observations);
	Ohat.back().push_back(1.0); // add 1 for bias

	for (int connection=0; connection<connections(); connection++){
		Ohat.push_back(matrixMultiply(Ohat[connection],Wbar[connection])); // Compute outputs
		sigmoid(Ohat.back());

		// D stuff
		D.push_back(matrix2d());
		int k = Ohat.back().size(); // number of hidden/output units, excluding bias
		for (int i=0; i<k; i++){ // add for last entry
			// For last output given, calculate the derivatives
			double Oi = Ohat.back().at(i);
			D[connection].push_back(matrix1d(k,0.0));
			D[connection][i][i] = Oi*(1-Oi); // create a diagonal matrix
		}

		Ohat.back().push_back(1.0);
	}
};

matrix2d NeuralNet::matrixMultiply(matrix2d &A, matrix2d &B){
	// returns a size(A,1)xsize(B,2) matrix
	//printf("mm");
	cmp_int_fatal(A[0].size(), B.size());

	matrix2d C(A.size());
	for (int row=0;	row<A.size(); row++){
		C[row] = matrix1d(B[0].size(),0.0);
		for (int col=0; col<B[0].size(); col++){
			for (int inner=0; inner<B.size(); inner++){
				C[row][col] += A[row][inner]*B[inner][col];
			}
		}
	}

	return C;
};

matrix2d NeuralNet::matrixMultiply(matrix1d &A, matrix1d &B){
	// returns a A.size()xB.size() matrix

	matrix2d C(A.size());
	for (int row=0;	row<A.size(); row++){
		C[row] = matrix1d(B.size(),0.0);
		for (int col=0; col<B.size(); col++){
			C[row][col] += A[row]*B[col];
		}
	}

	return C;
};

matrix1d NeuralNet::matrixMultiply(matrix2d &A, matrix1d &B){
	// returns a size(A,1)x1 matrix
	// assumes B is a COLUMN vector
	//printf("mm1");
	cmp_int_fatal(A[0].size(),B.size());

	matrix1d C(A.size(),0.0);
	for (int row=0;	row<A.size(); row++){
		for (int inner=0; inner<B.size(); inner++){
			C[row] += A[row][inner]*B[inner];
		}
	}

	return C;
};

matrix1d NeuralNet::matrixMultiply(matrix1d &A, matrix2d &B){
	// Use this if expecting to get a vector back;
	// assumes A is a ROW vector (1xcols)
	// returns a 1xsize(B,2) matrix

	//printf("Asize=%i,Bsize=%i",A.size(),B.size());
	//fprintf("mm2");
	cmp_int_fatal(A.size(),B.size());

	// MODIFY TO MATCH1
	matrix1d C(B[0].size(),0.0);
	for (int col=0; col<B[0].size(); col++){
		for (int inner=0; inner<B.size(); inner++){
			C[col] += A[inner]*B[inner][col];
		}
	}

	return C;
};

void  NeuralNet::matrixMultiply(matrix1d &A, matrix2d &B, matrix1d &C){
	// This fills C up to the size of B[0].size(). C is allowed to be larger by 1, to accommodate bias
	// Use this if expecting to get a vector back;
	// assumes A is a ROW vector (1xcols)
	// returns a 1xsize(B,2) matrix

	cmp_int_fatal(A.size(),B.size());
	if (B[0].size()!=C.size() && B[0].size()!=C.size()-1){
		printf("B and C sizes don't match. pausing");
		system("pause");
	}

	// MODIFY TO MATCH1
	for (int col=0; col<B[0].size(); col++){
		C[col] = 0.0;
		for (int inner=0; inner<B.size(); inner++){
			C[col] += A[inner]*B[inner][col];
		}
	}
};

void NeuralNet::sigmoid(matrix1d &myVector){
	for (int i=0; i<myVector.size(); i++){
		myVector[i] = 1/(1+exp(-myVector[i]));
	}
}

void NeuralNet::cmp_int_fatal(int a, int b){
	if (a!=b){
		printf("Ints do not match! Pausing to debug then exiting.");
		system("pause");
		exit(1);
	}
}
