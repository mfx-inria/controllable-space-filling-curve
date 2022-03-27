#include <array>
#include <vector>
#include <queue>

// 1 indexed
template <typename L>
struct Matching {
	int N, S;
	std::vector<L> G;
	std::vector<int> mate, base, seen;
	std::vector<std::pair<int,int>> label;
	
	Matching(int N): N(N), G(N+1), mate(N+1), base(N+1), label(N+1), seen(N+1) {}

	int group(int x) {
		if(seen[base[x]] == S) base[x] = group(base[x]);
		return base[x];
	}

	void match(int a, int b) {
		std::swap(b,mate[a]); if(mate[b] != a) return;
		if(!label[a].second) match(mate[b] = label[a].first, b); // vertex label
		else match(label[a].first, label[a].second), match(label[a].second, label[a].first); // edge label
	}

	bool augment(int st) {
		seen[st] = S; base[st] = 0; label[st] = {0,0};
		std::queue<int> q; q.push(st);
		while(!q.empty()) {
			int a = q.front(); q.pop();
			for(int b : G[a]) {
				if(seen[b] == S) {
					int x = group(a), y = group(b), lca = 0;
					while(x || y) {
						if(y) std::swap(x,y);
						if(label[x] == std::make_pair(a,b)) { lca = x; break; }
						label[x] = {a,b};
						x = group(label[mate[x]].first);
					}
					for(int v: {group(a), group(b)}) while(v != lca) {
						q.push(v);
						seen[v] = S;
						base[v] = lca;
						v = group(label[mate[v]].first);
					}
				} else if(!mate[b]) {
					match(mate[b] = a, b);
					return true;
				} else if(seen[mate[b]] != S) {
					seen[mate[b]] = S;
					base[mate[b]] = b;
					label[b] = {0,0};
					label[mate[b]] = {a,0};
					q.push(mate[b]);
				}
			}
		}
		return false;
	}

	int solve() {
		int ans = 0;
		for(int st = 1; st <= N; ++st) if(!mate[st]) ans += augment(S = st);
		return ans;
	}
};
