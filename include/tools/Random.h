#include <random>

template<typename T>
struct UniformInt {
public:
	UniformInt(T a, T b): a(a), r(b-a) {}

private:
	T a, r;
	
	template<typename U2, typename G, typename U>
	static U gen(G& g, U range) {
		U2 prod = U2(g()) * U2(range);
		U low = U(prod);
		if(low < range) {
			U threshold = -range % range;
			while(low < threshold) {
				prod = U2(g()) * U2(range);
				low = U(prod);
			}
		}
		return prod >> std::numeric_limits<U>::digits;
	}

public:
	template<typename G>
	T operator()(G &g) {
		typedef typename std::common_type<typename std::make_unsigned<T>::type, typename G::result_type>::type Ctype;
		constexpr Ctype gmin = G::min();
		constexpr Ctype gmax = G::max();
		constexpr Ctype grange = gmax - gmin;
		const Ctype range = r;

		Ctype x;
		if(grange > range) {
			const Ctype range2 = range + 1;
#if __SIZEOF_INT128__
			if constexpr (grange == std::numeric_limits<uint64_t>::max())
				return gen<__uint128_t>(g, (uint64_t) range2) + a;
#endif
			if constexpr (grange == std::numeric_limits<uint32_t>::max())
				return gen<uint64_t>(g, (uint32_t) range2) + a;
			const Ctype scaling = grange / range2;
			const Ctype lim = range2 * scaling;
			do x = Ctype(g()) - gmin;
			while(x >= lim);
			x /= scaling;
	  	} else if(grange < range) {
			const Ctype grange2 = grange + 1;
			const T a0 = a, r0 = r;
			a = 0;
			r = range / grange2;
			Ctype tmp;
			do {
				tmp = grange2 * operator()(g);
				x = tmp + (Ctype(g()) - gmin);
			} while(x > range || x < tmp);
			a = a0;
			r = r0;
		} else x = Ctype(g()) - gmin;

		return x + a;
	}
};
