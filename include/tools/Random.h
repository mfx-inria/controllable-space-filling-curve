#include <random>

namespace RandomImpl {
template<typename T, typename G> T genInt(G &g, T r);
template<typename T, typename G> T genReal(G &g);
}

template<typename T>
struct UniformInt {
	T a, r;
	UniformInt(T a, T b): a(a), r(b-a) {}
	template<typename G> T operator()(G &g) { return RandomImpl::genInt(g, r) + a; }
};

template<typename T>
struct UniformReal {
	T a, r;
	UniformReal(T a, T b): a(a), r(b-a) {}
	template<typename G> T operator()(G &g) { return RandomImpl::genReal<T>(g) * r + a; }
};
#include <iostream>
template<typename It, typename G>
void shuffle(It first, It last, G& g) {
	if (first == last) return;
	typedef typename std::make_unsigned<typename std::iterator_traits<It>::difference_type>::type Dist;
	typedef typename std::common_type<typename G::result_type, Dist>::type Ctype;

	constexpr Ctype grange = g.max() - g.min();
	const Ctype range = Ctype(last - first);

	if(grange / range >= range) {
		It it = first + 1;
		if((range % 2) == 0) std::iter_swap(it++, first + RandomImpl::genInt<Dist>(g, 1));
		Ctype swap_range = Ctype(it - first);
		while(it != last) {
			const Ctype ab = RandomImpl::genInt(g, swap_range * (swap_range+1) - 1);
			std::iter_swap(it++, first + (ab % swap_range));
			std::iter_swap(it++, first + (ab / swap_range));
			swap_range += 2;
		}
		return;
	}
	Dist swap_range = 0;
	for(It it = first + 1; it != last; ++it)
		std::iter_swap(it, first + RandomImpl::genInt(g, ++swap_range));
}

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
T genInt(G &g, T r) {
	typedef typename std::common_type<typename std::make_unsigned<T>::type, typename G::result_type>::type Ctype;
	constexpr Ctype gmin = G::min();
	constexpr Ctype gmax = G::max();
	constexpr Ctype grange = gmax - gmin;
	const Ctype range = r;

	if(grange > range) {
		const Ctype range2 = range + 1;
#		if __SIZEOF_INT128__
			if constexpr (grange == std::numeric_limits<uint64_t>::max())
				return genAux<__uint128_t>(g, (uint64_t) range2);
#		endif
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
			tmp = grange2 * genInt(g, r2);
			x = tmp + (Ctype(g()) - gmin);
		} while(x > range || x < tmp);
		return x;
	}
	return Ctype(g()) - gmin;
}

template<typename T, typename G>
T genReal(G &g) {
	constexpr int d = std::numeric_limits<T>::digits;
	constexpr long double r = static_cast<long double>(g.max()) - static_cast<long double>(g.min()) + 1.0L;
	constexpr int log2r = std::log2(r);
	constexpr int m = std::max<int>(1, (d + log2r - 1UL) / log2r);
	T sum = 0, tmp = 1;
	for(int k = m; k; --k) {
		sum += T(g() - g.min()) * tmp;
		tmp *= r;
	}
	T x = sum / tmp;
	if(x >= T(1)) [[unlikely]] x = std::nextafter(T(1), T(0));
	return x;
}

}