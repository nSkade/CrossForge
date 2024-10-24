/*  This file is part of the Pinocchio automatic rigging library.
    Copyright (C) 2007 Ilya Baran (ibaran@mit.edu)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef UTILS_H_INCLUDED
#define UTILS_H_INCLUDED

#include <istream>
#include <sstream>

namespace nsPinocchio {
template <class T>
inline string toString(const T& obj) {
 ostringstream stream;
 stream << obj;
 return stream.str();
}

inline vector<string> readWords(istream &stream)
{
    string whitespace = " \n\t\r";
    stream >> noskipws;
    
    char tmp[10000];
    stream.getline(tmp, 9990);
    string line(tmp);

    if(line.size() == 0)
        return vector<string>();

    while(line[line.size() - 1] == '\\') {
        line[line.size() - 1] = ' ';
        stream.getline(tmp, 9990);
        line = line + string(tmp);
    }
        
    //split the line into words
    vector<string> words;
    string::size_type pos = 0;
    while(pos != string::npos) {
        pos = line.find_first_not_of(whitespace, pos);
        if(pos == string::npos)
            break;
        string::size_type eow = line.find_first_of(whitespace, pos);
        words.push_back(string(line, pos, eow - pos));
        pos = eow;
    }
    
    return words;
}
}

#endif //UTILS_H_INCLUDED
