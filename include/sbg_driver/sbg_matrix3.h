/*!
*  \file         sbg_matrix3.h
*  \author       SBG Systems
*  \date         13/03/2020
*
*  \brief        Handle a 3x3 matrix.
*
*  SBG Systems Ros driver needs some basic matrix operations.
*  Ros uses Eigen for mathematical computations but to avoid dependency on
*  Eigen we chose to implement a basic custom matrix class with basic
*  mathematical operations needed.
*  This class also defines SbgMatrix3f and SbgMatrix3d for floats and doubles.
*
*  \section CodeCopyright Copyright Notice
*  MIT License
*
*  Copyright (c) 2023 SBG Systems
*
*  Permission is hereby granted, free of charge, to any person obtaining a copy
*  of this software and associated documentation files (the "Software"), to deal
*  in the Software without restriction, including without limitation the rights
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*  copies of the Software, and to permit persons to whom the Software is
*  furnished to do so, subject to the following conditions:
*
*  The above copyright notice and this permission notice shall be included in all
*  copies or substantial portions of the Software.
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
*  SOFTWARE.
*/

#ifndef SBG_MATRIX_3_H
#define SBG_MATRIX_3_H

// Sbg headers
#include <sbgDefines.h>
#include "sbg_vector3.h"

namespace sbg
{

/*!
 *
 */
template <class T>
class SbgMatrix3
{
private:

  //---------------------------------------------------------------------//
  //- Private variables                                                 -//
  //---------------------------------------------------------------------//

  std::array<T, 9> data_;

public:

  //---------------------------------------------------------------------//
  //- Constructor                                                       -//
  //---------------------------------------------------------------------//

  /*!
   * Empty constructor.
   */
  SbgMatrix3()
  {
    data_[0] = static_cast<T>(0.0);
    data_[1] = static_cast<T>(0.0);
    data_[2] = static_cast<T>(0.0);
    data_[3] = static_cast<T>(0.0);
    data_[4] = static_cast<T>(0.0);
    data_[5] = static_cast<T>(0.0);
    data_[6] = static_cast<T>(0.0);
    data_[7] = static_cast<T>(0.0);
    data_[8] = static_cast<T>(0.0);
  }

  /*!
   * Constructor.
   *
   * \param[in] value00                   Matrix X value.
   * \param[in] value01                   Matrix Y value.
   * \param[in] value02                   Matrix Z value.
   * \param[in] value10                   Matrix X value.
   * \param[in] value11                   Matrix Y value.
   * \param[in] value12                   Matrix Z value.
   * \param[in] value20                   Matrix X value.
   * \param[in] value21                   Matrix Y value.
   * \param[in] value22                   Matrix Z value.
   */
  SbgMatrix3(T value00, T value01, T value02, T value10, T value11, T value12, T value20, T value21, T value22)
  {
    data_[0] = value00;
    data_[1] = value01;
    data_[2] = value02;
    data_[3] = value10;
    data_[4] = value11;
    data_[5] = value12;
    data_[6] = value20;
    data_[7] = value21;
    data_[8] = value22;
  };

  /*!
   * Constructor.
   *
   * \param[in] p_raw_data                  Pointer to data array.
   * \param[in] array_size                  Array size (Should be defined as 9).
   */
  SbgMatrix3(const T* p_raw_data, size_t array_size)
  {
    assert(array_size == 9);

    data_[0] = p_raw_data[0];
    data_[1] = p_raw_data[1];
    data_[2] = p_raw_data[2];
    data_[3] = p_raw_data[3];
    data_[4] = p_raw_data[4];
    data_[5] = p_raw_data[5];
    data_[6] = p_raw_data[6];
    data_[7] = p_raw_data[7];
    data_[8] = p_raw_data[8];
  };

  //---------------------------------------------------------------------//
  //- Parameters                                                        -//
  //---------------------------------------------------------------------//

  //---------------------------------------------------------------------//
  //- Operations                                                        -//
  //---------------------------------------------------------------------//

  /*!
  * Getter parenthesis operator.
  *
  * \param[in] i                  Line.
  * \param[in] j                  Column.
  * \return                       Element at index.
  */
  const T operator() (int i, int j) const
  {
    assert(i * 3 + j < 9);

    return data_[i * 3 + j];
  }

  /*!
   * Get the raw data of the sbgMatrix.
   *
   * \return                                Raw vector data.
   */
  const T *data() const
  {
    return static_cast<const T*>(data_.data());
  };

  const SbgVector3<T> operator*(const SbgVector3<T>& vect) const
  {
    T x = data_[0] * vect(0) + data_[1] * vect(1) + data_[2] * vect(2);
    T y = data_[3] * vect(0) + data_[4] * vect(1) + data_[5] * vect(2);
    T z = data_[6] * vect(0) + data_[7] * vect(1) + data_[8] * vect(2);

    return SbgVector3<T>(x, y, z);
  }

  void transpose()
  {
    T swap = data_[1];
    data_[1] = data_[3];
    data_[3] = swap;

    swap = data_[2];
    data_[2] = data_[6];
    data_[6] = swap;

    swap = data_[5];
    data_[5] = data_[7];
    data_[7] = swap;
  }

  /*!
  * Make DCM from euler angles vector
  *
  * \param[in] vect               Euler angles vector.
  * \param[in] x                  Quaternion x value.
  * \param[in] y                  Quaternion y value.
  * \param[in] z                  Quaternion z value.
  */
  void makeDcm(const SbgVector3f& euler)
  {
    float cr = cosf(euler(0));
    float sr = sinf(euler(0));
    float cp = cosf(euler(1));
    float sp = sinf(euler(1));
    float cy = cosf(euler(2));
    float sy = sinf(euler(2));

    data_[0] = cp * cy;
    data_[3] = cp * sy;
    data_[6] = -sp;

    data_[1] = (sr * sp * cy) - (cr * sy);
    data_[4] = (sr * sp * sy) + (cr * cy);
    data_[7] = sr * cp;

    data_[2] = (cr * sp * cy) + (sy * sr);
    data_[5] = (cr * sp * sy) - (sr * cy);
    data_[8] = cr * cp;

  }

  /*!
  * Make DCM from quaternion values
  *
  * \param[in] w                  Quaternion w value.
  * \param[in] x                  Quaternion x value.
  * \param[in] y                  Quaternion y value.
  * \param[in] z                  Quaternion z value.
  */
  void makeDcm(float w, float x, float y, float z)
  {
    float wx = w * x;
    float wy = w * y;
    float wz = w * z;
    float xy = x * y;
    float xz = x * z;
    float yz = y * z;

    data_[0] = (2 * powf(w, 2)) + (2 * powf(x, 2)) - 1;
    data_[3] = (2 * xy) + (2 * wz);
    data_[6] = (2 * xz) - (2 * wy);

    data_[1] = (2 * xy) - (2 * wz);
    data_[4] = (2 * powf(w, 2)) + (2 * powf(y, 2)) - 1;
    data_[7] = (2 * yz) + (2 * wx);

    data_[2] = (2 * wy) + (2 * xz);
    data_[5] = (2 * yz) - (2 * wx);
    data_[8] = (2 * powf(w, 2)) + (2 * powf(z, 2)) - 1;
  }

};


typedef SbgMatrix3<float>  SbgMatrix3f;
typedef SbgMatrix3<double> SbgMatrix3d;

}

#endif // SBG_MATRIX_3_H
