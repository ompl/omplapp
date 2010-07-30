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

/* Author: Ioan Sucan */

#ifndef OMPL_DATASTRUCTURES_GRID_
#define OMPL_DATASTRUCTURES_GRID_

#include <vector>
#include <iostream>
#include <cstdlib>
#include <boost/unordered_map.hpp>

namespace ompl
{
    
    /** \brief Representation of a simple grid */
    template <typename _T>
    class Grid
    {
    public:
	
	/// definition of a coordinate within this grid
	typedef std::vector<int> Coord;
	
	/// definition of a cell in this grid
	struct Cell
	{
	    /// the data we store in the cell
	    _T                  data;

	    /// the coordinate of the cell
	    Coord               coord;
	    
	    Cell(void)
	    {
	    }
	    
	    virtual ~Cell(void)
	    {
	    }
	};
	
	/// the datatype for arrays of cells 
	typedef std::vector<Cell*> CellArray;	
	

	/// the constructor takes the dimension of the grid as argument
       	explicit
	Grid(unsigned int dimension)
	{
	    setDimension(dimension);
	}
	
	/// destructor
	virtual ~Grid(void)
	{
	    freeMemory();
	}
	
	/// clear all cells in the grid
	virtual void clear(void)
	{
	    freeMemory();
	}
	
	/// return the dimension of the grid
	unsigned int getDimension(void) const
	{
	    return dimension_;
	}
	
	/// update the dimension of the grid; this should not be done
	/// unless the grid is empty
	void setDimension(unsigned int dimension)
	{
	    dimension_ = dimension;
	    maxNeighbors_ = 2 * dimension_;
	}

	/// check if a cell exists at the specified coordinate
	bool has(const Coord &coord) const
	{
	    return getCell(coord) != NULL;
	}
	
	/// get the cell at a specified coordinate
	Cell* getCell(const Coord &coord) const
	{ 
	    iterator pos = hash_.find(const_cast<Coord*>(&coord));
	    Cell *c = (pos != hash_.end()) ? pos->second : NULL;
	    return c;
	}	
	
	/// get the list of neighbors for a given cell 
	void    neighbors(const Cell* cell, CellArray& list) const
	{
	    Coord test = cell->coord;
	    neighbors(test, list);
	}
	
	/// get the list of neighbors for a given coordinate
	void    neighbors(const Coord& coord, CellArray& list) const
	{
	    Coord test = coord;
	    neighbors(test, list);
	}

	/// get the list of neighbors for a given coordinate
	void    neighbors(Coord& coord, CellArray& list) const
	{
	    list.reserve(list.size() + maxNeighbors_);
	    
	    for (int i = dimension_ - 1 ; i >= 0 ; --i)
	    {
		coord[i]--;
		
		iterator pos = hash_.find(&coord);
		Cell *cell = (pos != hash_.end()) ? pos->second : NULL;

		if (cell)
		    list.push_back(cell);
		coord[i] += 2;

		pos = hash_.find(&coord);
		cell = (pos != hash_.end()) ? pos->second : NULL;
		
		if (cell)
		    list.push_back(cell);
		coord[i]--;
	    }
	}
	
	/// Instantiate a new cell at given coordinates; optionally
	/// return the list of future neighbors.
	/// Note: this call only creates the cell, but does not add it to the grid.
	/// It however updates the neighbor count for neighboring cells
	virtual Cell* createCell(const Coord& coord, CellArray *nbh = NULL)
	{
	    Cell *cell = new Cell();
	    cell->coord = coord;	    
	    if (nbh)
		neighbors(cell->coord, *nbh);	    
	    return cell;
	}

	/// Remove a cell from the grid. If the cell has not been
	/// added to the grid, only update the neighbor list
	virtual bool remove(Cell *cell)
	{
	    if (cell)
	    {
		typename CoordHash::iterator pos = hash_.find(&cell->coord);
		if (pos != hash_.end())
		{
		    hash_.erase(pos);
		    return true;
		}
	    }
	    return false;
	}
	
	/// add an instantiated cell to the grid
	virtual void add(Cell *cell)
	{
	    hash_.insert(std::make_pair(&cell->coord, cell));
	}
	
	/// clear the memory occupied by a cell; do not call this function unless remove() was called first
	virtual void destroyCell(Cell *cell) const
	{
	    delete cell;
	}
	
	/// get the data stored in the cells we are aware of
	void getContent(std::vector<_T> &content) const
	{
	    for (iterator i = hash_.begin() ; i != hash_.end() ; i++)
		content.push_back(i->second->data);
	}
	
	/// get the set of coordinates where there are cells
	void getCoordinates(std::vector<Coord*> &coords) const
	{
	    for (iterator i = hash_.begin() ; i != hash_.end() ; i++)
		coords.push_back(i->first);
	}
	
	/// get the set of instantiated cells in the grid
	void getCells(CellArray &cells) const
	{
	    for (iterator i = hash_.begin() ; i != hash_.end() ; i++)
		cells.push_back(i->second);
	}
	
	/// print the value of a coordinate to a stream 
	void printCoord(Coord& coord, std::ostream &out = std::cout) const
	{
	    out << "[ ";
	    for (unsigned int i = 0 ; i < dimension_ ; ++i)
		out << coord[i] << " ";
	    out << "]" << std::endl;
	}

	/// check if the grid is empty
	bool empty(void) const
	{
	    return hash_.empty();
	}
	
	/// check the size of the grid 
	unsigned int size(void) const
	{
	    return hash_.size();	    
	}

    protected:

	/// free the allocated memory
	void freeMemory(void)
	{
	    CellArray content;
	    getCells(content);
	    hash_.clear();
	    
	    for (unsigned int i = 0 ; i < content.size() ; i++)
		delete content[i];	    
	}

	/// hash function for coordinates
	struct HashFunCoordPtr
	{
	    std::size_t operator()(const Coord* const s) const
	    { 
		unsigned long h = 0;
		for (int i = s->size() - 1; i >= 0; --i)
		{
		    int high = h & 0xf8000000;
		    h = h << 5;
		    h = h ^ (high >> 27);
		    h = h ^ s->at(i);
		}		
		return (std::size_t) h;
	    }
	};


	/// equality operator for coordinate pointers
	struct EqualCoordPtr
	{
	    bool operator()(const Coord* const c1, const Coord* const c2) const
	    {
		return *c1 == *c2;
	    }
	};
	
	/// define the datatype for the used hash structure
	typedef boost::unordered_map<Coord*, Cell*, HashFunCoordPtr, EqualCoordPtr> CoordHash;

    public:
	
	/// we only allow const iterators
	typedef typename CoordHash::const_iterator iterator;

	/// return the begin() iterator for the grid 
	iterator begin(void) const
	{
	    return hash_.begin();
	}
	
	/// return the end() iterator for the grid 
	iterator end(void) const
	{
	    return hash_.end();
	}
	
    protected:

	unsigned int     dimension_;
	unsigned int     maxNeighbors_;
	
	CoordHash        hash_;
    };
}

#endif
