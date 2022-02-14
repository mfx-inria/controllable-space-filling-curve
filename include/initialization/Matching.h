#include <array>
#include <vector>
#include <queue>

template <typename L>
struct Matching {
	struct Node {
		L link;
		int getMatch() const { return match; }
	private:
		friend Matching;
		int base, father, match;
		int seen, ancestor;
		bool inQ;
		void init(int i, int s) { base = i; father = -1; seen = s; inQ = false; }
	};
	
	Matching(int n): nodes(n), S(0) {}

	Node& node(int i) { return nodes[i]; }

	int compute() {
		int nmatch = 0;
		for(Node &n : nodes) n.match = n.seen = n.ancestor = -1;
		for(int i = 0; i < (int) nodes.size(); ++i) if(nodes[i].match == -1)
			nmatch += augment(i);
		return nmatch;
	}

private:
	std::vector<Node> nodes;
	std::queue<int> Q;
	int S;

	int augment(int i) {
		nodes[i].init(i, S = i);
		nodes[i].inQ = true;
		for(Q = std::queue<int>({i}); !Q.empty(); Q.pop()) {
			i = Q.front();
			Node &ni = nodes[i];
			for(int j : nodes[i].link) {
				Node &nj = nodes[j];
				if(nj.seen != S) nj.init(j, S);
				if(nj.base == ni.base || ni.match == j) continue;
				if(j == S) blossom(i, j);
				else {
					const int mj = nj.match;
					if(mj != -1) {
						if(nodes[mj].seen != S) nodes[mj].init(mj, S);
						else if(nodes[mj].father != -1) blossom(i, j);
					}
					if(nj.father == -1) {
						nj.father = i;
						if(mj == -1) {
							do {
								const int k = nodes[j].father;
								const int l = nodes[k].match;
								nodes[k].match = j;
								nodes[j].match = k;
								j = l;
							} while(j != -1);
							return 1;
						} else if(!nodes[mj].inQ) {
							nodes[mj].inQ = true;
							Q.push(mj);
						}
					}
				}
			}
		}
		return 0;
	}

	void blossom(int i, int j) {
		int lca = LCA(i, j);
		if(nodes[i].base != lca) contract(lca, i, j);
		if(nodes[j].base != lca) contract(lca, j, i);
	}

	int LCA(int i, int j) {
		static int A = 0; ++ A;
		while(true) {
			nodes[i=nodes[i].base].ancestor = A;
			if(i == S) break;
			i = nodes[nodes[i].match].father;
		}
		while(1) {
			if(nodes[j=nodes[j].base].ancestor == A) return j;
			j = nodes[nodes[j].match].father;
		}
	}

	void contract(int lca, int i, int j) {
		do {
			nodes[i].father = j;
			j = nodes[i].match;
			nodes[i].base = nodes[j].base = lca;
			if(!nodes[i].inQ) { nodes[i].inQ = true; Q.push(i); }
			if(!nodes[j].inQ) { nodes[j].inQ = true; Q.push(j); }
			i = nodes[j].father;
		} while(nodes[i].base != lca);
	}
};
