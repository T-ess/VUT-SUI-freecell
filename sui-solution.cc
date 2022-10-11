#include "search-strategies.h"
#include <iostream>
#include <deque>
#include <queue>
#include <set>
#include <map>
#include <algorithm>
#include "memusage.h"
using namespace std;

struct stateInfo {
	shared_ptr<SearchState> state;
	vector<shared_ptr<SearchAction>> actions;
};

struct stateInfoAstar {
	shared_ptr<SearchState> state;
	vector<shared_ptr<SearchAction>> actions;
	double g; //cost

	bool operator<(const struct stateInfoAstar& other) const {
	return g < other.g;
}
};


std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
	// variables
	deque<stateInfo> BFSopen;
	set<SearchState> BFSclosed;

	// initial state
	stateInfo initInfo = {make_shared<SearchState>(init_state), {}};
	BFSopen.push_front(move(initInfo));

	if (BFSopen.empty()) return {};
	while (!BFSopen.empty() && !BFSopen.back().state->isFinal()) {
		shared_ptr<SearchState> actState = move(BFSopen.back().state);
		vector<shared_ptr<SearchAction>> actStateActions = move(BFSopen.back().actions);
		if (BFSclosed.find(*actState) != BFSclosed.end()) { // current state already visited
			BFSopen.pop_back();
			continue;
		}
		BFSclosed.insert(*actState);
		BFSopen.pop_back();
		// expand current state and push new states
		vector<SearchAction> currAct = actState->actions();
		for (auto &gameMove : currAct) {
			if ((getCurrentRSS() + 3000000) >= mem_limit_) {
				cout << "error" << endl;
				return {}; // memory check
			}
			shared_ptr<SearchState> newStatePtr = make_shared<SearchState>(move(gameMove.execute(*actState)));
			if(BFSclosed.find(*newStatePtr) != BFSclosed.end()) continue; // state found in CLOSED
			vector<shared_ptr<SearchAction>> actionPath = actStateActions;
			actionPath.push_back(make_shared<SearchAction>(gameMove));
			BFSopen.push_front({move(newStatePtr), move(actionPath)});
		}
	}
	vector<SearchAction> finalActions = {};
	if (!BFSopen.empty() && BFSopen.back().state->isFinal()) {
		for (auto &act : BFSopen.back().actions) {
			finalActions.push_back(*act);
		}
		cout << "yay" << endl;
		printf("BFS: MemLimit %f%%    %lu / %lu\n", (double) getCurrentRSS() / mem_limit_ * 100, getCurrentRSS(), mem_limit_);
		return finalActions;
	}
	return {};
}

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState &init_state) {
	// variables
	deque<stateInfo> DFSopen;
	set<SearchState> DFSclosed;

	// initial state
	stateInfo initInfo = {make_shared<SearchState>(init_state), {}};
	DFSopen.push_back(move(initInfo));

	if (DFSopen.empty()) return {};
	while (!DFSopen.empty() && !DFSopen.back().state->isFinal()) {
		if (DFSopen.back().actions.size() < depth_limit_) {
			shared_ptr<SearchState> actState = move(DFSopen.back().state);
			vector<shared_ptr<SearchAction>> actStateActions = move(DFSopen.back().actions);
			if (DFSclosed.find(*actState) != DFSclosed.end()) { // current state already visited
				DFSopen.pop_back();
				continue;
			}
			DFSclosed.insert(*actState);
			DFSopen.pop_back();
			// expand current state and push new states
			vector<SearchAction> currAct = actState->actions();
			for (auto &gameMove : currAct) {
				if ((getCurrentRSS() + 3000000) >= mem_limit_) {
					cout << "error" << endl;
					return {}; // memory check
				}
				shared_ptr<SearchState> newStatePtr = make_shared<SearchState>(move(gameMove.execute(*actState)));
				if(DFSclosed.find(*newStatePtr) != DFSclosed.end()) continue; // state found in CLOSED
				vector<shared_ptr<SearchAction>> actionPath = actStateActions;
				actionPath.push_back(make_shared<SearchAction>(gameMove));
				DFSopen.push_back({move(newStatePtr), move(actionPath)});
			}
		} else {
			DFSopen.pop_back();
		}
	}
	vector<SearchAction> finalActions = {};
	if (!DFSopen.empty() && DFSopen.back().state->isFinal()) {
		for (auto &act : DFSopen.back().actions) {
			finalActions.push_back(*act);
		}
		cout << "yay" << endl;
		printf("DFS: MemLimit %f%%    %lu / %lu\n", (double) getCurrentRSS() / mem_limit_ * 100, getCurrentRSS(), mem_limit_);
		return finalActions;
	}
	return {};
}

double StudentHeuristic::distanceLowerBound(const GameState &state) const {
    return 0;
}

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {
	// variables
	set<SearchState> Aclosed;
	priority_queue<stateInfoAstar> Apriority;
	// g = movement cost from starting state
	// h = heuristic
	double g = 0, h;
	
	// initial state
	stateInfoAstar initInfoAstar = {make_shared<SearchState>(init_state), {}, g};
	Apriority.push(move(initInfoAstar)); //priority queue

	if (Apriority.empty()) return {};
	while (!Apriority.empty() && !Apriority.top().state->isFinal()) {
		shared_ptr<SearchState> actState = move(Apriority.top().state);
		vector<shared_ptr<SearchAction>> actStateActions = move(Apriority.top().actions);
		if (Aclosed.find(*actState) != Aclosed.end()) { // current state already visited
			Apriority.pop();
			continue;
		}
		Aclosed.insert(*actState);
		Apriority.pop();
		// expand current state and push new states
		vector<SearchAction> currAct = actState->actions();
		for (auto &gameMove : currAct) {
			if ((getCurrentRSS() + 3000000) >= mem_limit_) {
				cout << "error" << endl;
				return {}; // memory check
			}
			shared_ptr<SearchState> newStatePtr = make_shared<SearchState>(move(gameMove.execute(*actState)));
			
			h = compute_heuristic(*newStatePtr, *heuristic_); 

			if(Aclosed.find(*newStatePtr) != Aclosed.end()) continue; // state found in CLOSED
			vector<shared_ptr<SearchAction>> actionPath = actStateActions;
			actionPath.push_back(make_shared<SearchAction>(gameMove));
			g = actionPath.size(); //cost of state
			Apriority.push({move(newStatePtr), move(actionPath), g + h});
		}
	}
	vector<SearchAction> finalActions = {};
	if (!Apriority.empty() && Apriority.top().state->isFinal()) {
		for (auto &act : Apriority.top().actions) {
			finalActions.push_back(*act);
		}
		cout << "yay" << endl;
		printf("A_star: MemLimit %f%%    %lu / %lu\n", (double) getCurrentRSS() / mem_limit_ * 100, getCurrentRSS(), mem_limit_);
		return finalActions;
	}
	return {};


	return {};
}