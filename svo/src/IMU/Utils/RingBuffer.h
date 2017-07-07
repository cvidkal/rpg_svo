#pragma once

// Ring Buffer
template<typename T, int N>
class RingBuffer
{
protected:
	int head;
	int tail;
	T _buffer[1 << N];  // 2^N
	const int mBufferSize;
private:
	// mBufferSize_1 = mBufferSize - 1
	const int mBufferSize_1;

public:
	RingBuffer();
	~RingBuffer();

	//-------------------------------------------------------------------------
	// check whether buffer is empty
	inline bool Empty() const {
		return (head == tail);
	}

	//-------------------------------------------------------------------------
	// check whether buffer is full
	inline bool Full() const {
		return ((head + 1) & mBufferSize_1) == tail;
	}

	//-------------------------------------------------------------------------
	// calculate data length in buffer
	inline int Size() const {
		return (head + mBufferSize - tail) & mBufferSize_1;
	}

	//-------------------------------------------------------------------------
	// clear buffer, only need to reset head and tail index
	inline void Clear() {
		head = 0;
		tail = 0;
	}

	//-------------------------------------------------------------------------
	// calculate index from start pos, with increment delta
	inline int Index(int start, int delta) const {
		// no matter delta < 0
		return (start + mBufferSize + delta) & mBufferSize_1;
	}

	//-------------------------------------------------------------------------
	// get head index
	inline int getHeadIndex() const {
		return Index(head, -1);
	}

	//-------------------------------------------------------------------------
	// get head node
	inline const T& getHeadNode() {
		return _buffer[Index(head, -1)];
	}

	//-------------------------------------------------------------------------
	// get node by delta index from head
	inline const T& getNodeByDeltaIndex(int delta)
	{
		int index; 
		if (delta > 0)
			index = Index(head, -delta);
		else
			index = Index(head, delta);

		return _buffer[index];
	}

protected:
	

	//-------------------------------------------------------------------------
	// calculate delta size from start index to end index
	inline int DeltaSize(int start, int end) const {
		return (end + mBufferSize - start + 1) & mBufferSize_1;
	}

	//-------------------------------------------------------------------------
	// head index go ahead when add data to buffer
	// todo: make it atomic
	inline void IndexGoAhead() {
		head = (head + 1) & mBufferSize_1;
		// when full, tail go ahead
		if (head == tail) {
			tail = (tail + 1) & mBufferSize_1;
		}
	}
};

//-----------------------------------------------------------------------------
template<typename T, int N>
RingBuffer<T, N>::RingBuffer()
	: mBufferSize(1 << N), mBufferSize_1(mBufferSize - 1)
{
	Clear();
}

//-----------------------------------------------------------------------------
template<typename T, int N>
RingBuffer<T, N>::~RingBuffer()
{
}


