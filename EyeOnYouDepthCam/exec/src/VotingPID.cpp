#include "VotingPID.h"

vector< vector<string> > VotingPID::myPIDVector(15,vector<string>(votingLength, "") );
int VotingPID::ID = -1;
string VotingPID::nameVoting[15] = {""};

//VotingPID::VotingPID(vector<string> init_myPIDVector, string init_id, string init_nameVoting)
//{
//	id = init_id;
//	for (int i = 0; i < 6; i++) {	
//		nameVoting[i] = init_nameVoting;
//		myPIDVector.push_back(init_myPIDVector);
//	}
//}

pair<string, unsigned int> VotingPID::get_max(const map<string, unsigned int>& x)
{
	using pairtype = pair<string, unsigned int>;
	return *max_element(x.begin(), x.end(), [](const pairtype & p1, const pairtype & p2) {
		return p1.second < p2.second;
	});
}

string VotingPID::modeOfVector(const vector<string>& vals)
{
	map<string, unsigned int> rv;

	for (auto val = vals.begin(); val != vals.end(); ++val) {
		rv[*val]++;
	}

	auto max = get_max(rv);

	return max.first;
}

string VotingPID::votingOfPID(int rec_id, string rec_name)
{
	if (myPIDVector[rec_id].size() == votingLength)
		myPIDVector[rec_id].erase(myPIDVector[rec_id].begin());

	myPIDVector[rec_id].push_back(rec_name);

	//cout << "myPIDVector[myPIDID].size()" << myPIDVector[rec_id].size() << endl;
	return modeOfVector(myPIDVector[rec_id]);
}

int VotingPID::getID()
{
	return ID;
}
void VotingPID::setID(string rec_id)
{
	ID = stoi(rec_id);
}

string VotingPID::getnameVotingWithIndex(int rec_index)
{
	return nameVoting[rec_index];
}
void VotingPID::setnameVotingWithIndex(int rec_index, string rec_VotingResult)
{
	nameVoting[rec_index] = rec_VotingResult;
}
void VotingPID::cleanVotingBufferWithIndex(int rec_id)
{
	myPIDVector[rec_id].clear();
	cout << "=======Voting buffer cleaned=======";
}