/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* \author Ioan Sucan */

#ifndef OMPL_DATASTRUCTURES_BINARY_HEAP_
#define OMPL_DATASTRUCTURES_BINARY_HEAP_

#include <functional>
#include <vector>
#include <cassert>

namespace ompl
{
    
    /** This class provides an implementation of an updatable
	min-heap. Using it is a bit cumbersome, as it requires keeping
	track of the Element* type, however, it should be as fast as
	it gets with an updatable heap. */
    template <typename _T, 
	      class LessThan = std::less<_T> >
    class BinaryHeap
    {
    public:
	
        class Element
	{
	    friend class BinaryHeap;
	private:
	    unsigned int position;
	public:
	    _T           data;
	};
	
	typedef void (*EventAfterInsert) (Element*, void*);
	typedef void (*EventBeforeRemove)(Element*, void*);
	
	BinaryHeap(void)
	{
	    eventAfterInsert_  = NULL;
	    eventBeforeRemove_ = NULL;
	}
	
	~BinaryHeap(void)
	{
	    clear();
	}
	
	void onAfterInsert(EventAfterInsert event, void *arg)
	{
	    eventAfterInsert_ = event;
	    eventAfterInsertData_ = arg;
	}

	void onBeforeRemove(EventBeforeRemove event, void *arg)
	{
	    eventBeforeRemove_ = event;
	    eventBeforeRemoveData_ = arg;
	}	

	void clear(void)
	{
	    for (typename std::vector<Element*>::iterator i = vector_.begin() ;
		 i != vector_.end() ; i++)
		delete *i;
	    vector_.clear();
	}
	
	Element* top(void) const
	{
	    return vector_.empty() ? NULL : vector_.at(0);
	}

	void pop(void)
	{
	    removePos(0);
	}
	
	void remove(Element* element)
	{
	    if (eventBeforeRemove_)
		eventBeforeRemove_(element, eventBeforeRemoveData_);
	    removePos(element->position);
	}
	
	Element* insert(const _T& data)
	{
	    Element* element = new Element();
	    element->data = data;
	    const unsigned int pos = vector_.size();
	    element->position = pos;	    
	    vector_.push_back(element);
	    percolateUp(pos);
	    if (eventAfterInsert_)
		eventAfterInsert_(element, eventAfterInsertData_);
	    return element;
	}
	
	void insert(const std::vector<_T>& list)
	{
	    const unsigned int n = vector_.size();
	    const unsigned int m = list.size();
	    for (unsigned int i = 0 ; i < m ; i++)
	    {
		const unsigned int pos = i + n;
		Element* element = newElement(list[i], pos);
		vector_.push_back(element);
		percolateUp(pos);
		if (eventAfterInsert_)
		    eventAfterInsert_(element, eventAfterInsertData_);
	    }
	}
	
	void buildFrom(const std::vector<_T>& list)
	{
	    const unsigned int m = list.size();
	    for (unsigned int i = 0 ; i < m ; i++)
		vector_.push_back(newElement(list[i], i));
	    build();
	}

	void rebuild(void)
	{	    
	    build();
	}

	void update(Element* element)
	{
	    const unsigned int pos = element->position;
	    assert(vector_[pos] == element);
	    percolateUp(pos);
	    percolateDown(pos);
	}
	
	bool empty(void) const
	{
	    return vector_.empty();
	}
	
	unsigned int size(void) const
	{
	    return vector_.size();
	}
	
	void getContent(std::vector<_T> &content) const
	{
	    for (typename std::vector<Element*>::const_iterator i = vector_.begin();
		 i != vector_.end() ; i++)
		content.push_back((*i)->data);
	}
	
	void sort(std::vector<_T>& list)
	{	    
	    const unsigned int n         = list.size();
	    std::vector<Element*> backup = vector_;
	    vector_.clear();
	    for (unsigned int i = 0 ; i < n ; i++)
		vector_.push_back(newElement(list[i], i));
	    build();
	    list.clear();
	    list.reserve(n);
	    
	    for (unsigned int i = 0 ; i < n ; i++)
	    {
		list.push_back(vector_[0]->data);
		removePos(0);
	    }
	    vector_ = backup;
	}
	
    private:

        LessThan                 lt_;
	
	std::vector<Element*>    vector_;

	EventAfterInsert         eventAfterInsert_;
	void                    *eventAfterInsertData_;
	EventBeforeRemove        eventBeforeRemove_;
	void                    *eventBeforeRemoveData_;
	
	void removePos(unsigned int pos)
	{
	    const int n = vector_.size() - 1;
	    delete vector_[pos];
	    if ((int)pos < n)
	    {
		vector_[pos] = vector_.back();
		vector_[pos]->position = pos;
		vector_.pop_back();
		percolateDown(pos);
	    }
	    else
		vector_.pop_back();
	}
	
	Element* newElement(_T& data, unsigned int pos) const
	{
	    Element* element = new Element();
	    element->data = data;
	    element->position = pos;
	    return element;	    
	}

	void build(void)
	{
	    for(int i = vector_.size() / 2 - 1; i >= 0; --i)
		percolateDown(i);
	}

	void percolateDown(const unsigned int pos)
	{
	    const unsigned int n      = vector_.size();
	    Element*           tmp    = vector_[pos];
	    unsigned int       parent = pos;
	    unsigned int       child  = (pos + 1) << 1;
	    
	    while (child < n)
	    {
		if (lt_(vector_[child - 1]->data, vector_[child]->data)) child--;
		if (lt_(vector_[child]->data,  tmp->data))
		{
		    vector_[parent] = vector_[child];
		    vector_[parent]->position = parent;
		}		
		else
		    break;
		parent = child;
		child  = (child + 1) << 1;
	    }
	    if (child == n)
	    {
		child--;
		if (lt_(vector_[child]->data, tmp->data))
		{
		    vector_[parent] = vector_[child];
		    vector_[parent]->position = parent;
		    parent = child;
		}
	    }
	    if (parent != pos)
	    {
		vector_[parent] = tmp;
		vector_[parent]->position = parent;
	    }
	}

	void percolateUp(const unsigned int pos)
	{
	    Element*           tmp    = vector_[pos];
	    unsigned int       child  = pos;
	    unsigned int       parent = (pos - 1) >> 1;
	    
	    while (child > 0 && lt_(tmp->data, vector_[parent]->data))
	    {
		vector_[child] = vector_[parent];
		vector_[child]->position = child;
		child  = parent;
		parent = (parent - 1) >> 1;
	    }
	    if (child != pos)
	    {
		vector_[child] = tmp;
		vector_[child]->position = child;
	    }
	}
    };

}

#endif
