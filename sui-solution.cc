#include "search-strategies.h"
#include <iostream>
#include <deque>
#include <set>
#include <map>
#include <algorithm>
using namespace std;

bool operator==(const SearchState &a, const SearchState &b) {
    return a.state_ == b.state_;
}

std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
	return {};
}

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState &init_state) {
	// variables
	deque<shared_ptr<SearchState>> DFSopen;
	set<shared_ptr<SearchState>> DFSclosed;
	map<shared_ptr<SearchState>,pair<shared_ptr<SearchState>,shared_ptr<SearchAction>>> DFSactions; // actual state > parent + action

	// initial state initialization
	DFSopen.push_back(make_shared<SearchState>(init_state));
	DFSactions.insert(make_pair(make_shared<SearchState>(init_state), make_pair(nullptr, nullptr)));

	if (DFSopen.empty()) return {};
	while (!DFSopen.empty() && !DFSopen.back().get()->isFinal()) {
		//TODO depth check
		shared_ptr<SearchState> actState = DFSopen.back();
		DFSclosed.insert(actState);
		DFSopen.pop_back();
		// expand and push new states
		vector<SearchAction> currAct = actState.get()->actions();
		for (auto &move : currAct) {
			SearchState newState = move.execute(*(actState.get()));
			// check if newState should be in stack (is not in open or closed)
			auto itr = DFSclosed.find(make_shared<SearchState>(newState));
			if(itr != DFSclosed.end()) {
				// state was found in closed
				continue;
			}
			auto itr2 = find(DFSopen.begin(), DFSopen.end(), make_shared<SearchState>(newState));
			if(itr2 != DFSopen.end()) {
				// state was found in open
				continue;
			}

			// push the new state
			DFSopen.push_back(make_shared<SearchState>(newState));
			// save the parent and action of new state
			DFSactions.emplace(make_shared<SearchState>(newState), make_pair(actState, make_shared<SearchAction>(move)));
		}
	}

	if (DFSopen.empty()) {
		return {};
	} else if (DFSopen.back().get()->isFinal()) {
		vector<SearchAction> finalActions;
		// get vector of actions that led to the final state
		shared_ptr<SearchState> state = DFSopen.back();

		while (true) {
			if (DFSactions.find(state) == DFSactions.end()) {
				// invalid
				return {};
			} else {
				if (DFSactions[state].second == nullptr) {
					// the initial state
					return finalActions;
				} else {
					// insert the next action to the beginning of the vector
					finalActions.push_back(*(DFSactions[state].second.get()));
					rotate(finalActions.rbegin(), finalActions.rbegin() + 1, finalActions.rend()); // push_front
					state = DFSactions[state].first;
				}
			}
		}

	}
	return {};
}

double StudentHeuristic::distanceLowerBound(const GameState &state) const {
    return 0;
}

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {
	return {};
}
