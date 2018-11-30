#ifndef VOTINGPID_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define VOTINGPID_H

#include <string>
#include <map>
#include <vector>
#include <iostream>
#include <algorithm>
using namespace std;

class VotingPID {
	protected:
		// PIDVector for voting
		static const int votingLength = 5;
		static vector< vector<string> > myPIDVector;
		static int ID;
		static string nameVoting[6];

	public:
		//VotingPID(vector<string> init_myPIDVector, string init_id, string init_nameVoting);
		static pair<string, unsigned int> get_max(const map<string, unsigned int>& x);
		static string modeOfVector(const vector<string>& vals);
		static string votingOfPID(int rec_id, string rec_name);
		static int getID();
		static void setID(string rec_id);
		static string getnameVotingWithIndex(int rec_index);
		static void setnameVotingWithIndex(int rec_index, string rec_VotingResult);
};

#endif