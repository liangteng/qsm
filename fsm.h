#ifndef  FSM_H
#define  FSM_H

#include <exception>
#include <unordered_map>
#include <thread>
#include <type_traits>
#include <vector>

// exceptions
// the current state is invalid
class invalid_state_error : public std::exception {
public:
    const char* what() const noexcept {
        return "invalid state";
    }
}; // class invalid_state_error

// more than one condition matches. the next state is ambiguous
class ambiguous_state_error : public std::exception {
public:
    const char* what() const noexcept {
        return "ambiguous states";
    }
}; // class ambiguous_state_error

// A general FSM model. C++11 compatible
// User registers states and transition conditions(function), init the FSM with initializing state,
// and call `trigger()` to run the FSM.
// NOTE NOT MT-safe
template<typename T>
class FSM {
    using trans_func_t = std::function<bool()>;
    using state_info_t = std::pair<T, trans_func_t>;
    using transition_table_t = std::unordered_map<T, std::vector<state_info_t> >;
    using edges_tbl_t = std::unordered_map<T, std::pair<int32_t/*in edges*/, int32_t/*out edges*/> >;
public:
    // constructor
    FSM();

    // desctructor
    ~FSM();

    // register state transitions and build the FSM.
    // The pred is transition condition from `pre_state` to `next_state`.
    template<typename Pr>
    bool register_transition(T pre_state, T next_state, Pr pred);

    // get current state
    T state() const;

    // initialize the FSM with the `init_state`.
    // calling `intialize()` means all states and transitions are settled. In another word, the
    // `initalize()` should be called only after all states and transitions are registerd.
    void set_state(T init_state);

    // check if the state is terminated state
    bool is_terminated_state(const T state) const;

    // trigger.
    // we check all transition conditions assiciated with current state by registered order, and if
    // one condition matches, we change the state and breakout.
    // if state changed, it returns true. Otherwise it returns false.
    void trigger(T& old_state, T& new_state);

    // try to trigger.
    bool try_trigger(T& old_state, T& new_state);

    // try to trigger until timed out or state changed.
    // `abs_time` is the absoluted time to block until
    // returning true means state changed, while false means a timeout occured or state not changed.
    bool timed_trigger(T& old_state, T& new_state, const struct timespec& abs_time);

private:
    // transition table
    transition_table_t _trans_tbl;

    // numbers of input and output edges of each state
    edges_tbl_t _edges_tbl;

    // the current state
    T _state;

}; // class FSM

// implement
template<typename T>
FSM<T>::FSM() {
}

template<typename T>
FSM<T>::~FSM() {
}

template<typename T> template<typename Pr>
bool FSM<T>::register_transition(T pre_state, T next_state, Pr pred) {

    if (pre_state == next_state) {
        // pre and next states should not be identical
        return false;
    }

    auto& next_si_vec = _trans_tbl[pre_state];

    if (std::any_of(next_si_vec.begin(), next_si_vec.end(),
                [&next_state](const state_info_t& si) {return si.first == next_state;})) {
        // duplicated. 
        return false;
    }

    next_si_vec.emplace_back(next_state, pred);

    // increment input and output edges
    if (_edges_tbl.find(pre_state) == _edges_tbl.end()) {
        _edges_tbl.emplace(pre_state, std::make_pair(0, 0));
    }
    if (_edges_tbl.find(next_state) == _edges_tbl.end()) {
        _edges_tbl.emplace(next_state, std::make_pair(0, 0));
    }
    ++_edges_tbl[pre_state].second;
    ++_edges_tbl[next_state].first;

    return true;
}

template<typename T>
T FSM<T>::state() const {
    return _state;
}

template<typename T>
void FSM<T>::set_state(T init_state) {
    _state = init_state;
}

template<typename T>
bool FSM<T>::is_terminated_state(const T state) const {
    auto&& edges_tbl_iter = _edges_tbl.find(state);
    if (edges_tbl_iter != _edges_tbl.end() && edges_tbl_iter->second.second == 0) {
        return true;
    }
    return false;
}

template<typename T>
void FSM<T>::trigger(T& old_state, T& new_state) {
    do {
        if (this->try_trigger(old_state, new_state)) {
            break;
        }
        std::this_thread::yield();
    } while (1);
}

template<typename T>
bool FSM<T>::try_trigger(T& old_state, T& new_state) {
    
    if (this->is_terminated_state(_state)) {
        // current state is terminated state, trapped
        old_state = _state;
        new_state = _state;
        return true;
    }

    auto&& iter = _trans_tbl.find(_state);
    if (iter == _trans_tbl.end()) {
        // current state is invalid. maybe the FSM was initialized with invalid state.
        throw invalid_state_error();
    }

    int32_t cnt = 0;
    T tmp_new_state = _state;
    for (auto& next_si : iter->second) {
        if (next_si.second()) {
            ++cnt;
            tmp_new_state = next_si.first;
        }
        if (cnt > 1) {
            throw ambiguous_state_error();
        }
    }

    if (cnt == 1) {
        old_state = _state;
        new_state = tmp_new_state;
        _state = tmp_new_state;
    }
    return cnt;
}

template<typename T>
bool FSM<T>::timed_trigger(T& old_state, T& new_state, const struct timespec& abs_time) {

    // timeout check function
    auto is_timeout_fn = [](const struct timespec& abs_time, const struct timespec& cur_time) {
        return cur_time.tv_sec > abs_time.tv_sec
            || (cur_time.tv_sec == abs_time.tv_sec && cur_time.tv_nsec >= abs_time.tv_nsec);
    };

    if (this->try_trigger(old_state, new_state)) {
        return true;
    }

    struct timespec cur_time = {0, 0};
    do {
        // call thread yield, offering the opportunity to reschedule.
        std::this_thread::yield();

        if (this->try_trigger(old_state, new_state)) {
            // we triggered the FSM
            return true;
        }

        if (clock_gettime(CLOCK_REALTIME, &cur_time) != 0) {
            // we get current timespec failed, so we can do nothing unless return false.
            return false;
        }

    } while (!is_timeout_fn(abs_time, cur_time));

    // a timeout occured.
    return false;
}

#endif  //FSM_H
