#include <random>

namespace RandomImpl {
template<typename T, typename G> T gen(G &g, T r);
}

template<typename T>
struct UniformInt {
	T a, r;
	UniformInt(T a, T b): a(a), r(b-a) {}
	template<typename G> T operator()(G &g) { return RandomImpl::gen(g, r) + a; }
};

namespace RandomImpl {

template<typename U2, typename G, typename U>
static U genAux(G& g, U range) {
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

template<typename T, typename G>
T gen(G &g, T r) {
	typedef typename std::common_type<typename std::make_unsigned<T>::type, typename G::result_type>::type Ctype;
	constexpr Ctype gmin = G::min();
	constexpr Ctype gmax = G::max();
	constexpr Ctype grange = gmax - gmin;
	const Ctype range = r;

	if(grange > range) {
		const Ctype range2 = range + 1;
#if __SIZEOF_INT128__
		if constexpr (grange == std::numeric_limits<uint64_t>::max())
			return genAux<__uint128_t>(g, (uint64_t) range2);
#endif
		if constexpr (grange == std::numeric_limits<uint32_t>::max())
			return genAux<uint64_t>(g, (uint32_t) range2);
		const Ctype scaling = grange / range2;
		const Ctype lim = range2 * scaling;
		Ctype x;
		do x = Ctype(g()) - gmin;
		while(x >= lim);
		return x / scaling;
	} else if(grange < range) {
		const Ctype grange2 = grange + 1;
		const Ctype r2 = range / grange2;
		Ctype tmp, x;
		do {
			tmp = grange2 * gen(g, r2);
			x = tmp + (Ctype(g()) - gmin);
		} while(x > range || x < tmp);
		return x;
	}
	return Ctype(g()) - gmin;
}

}