#ifndef SBG_MATRIX_3_H
#define SBG_MATRIX_3_H

// Sbg headers
#include <sbgDefines.h>

namespace sbg
{

/*!
 * Class to define a Matrix3.
 */
template <class T>
class SbgMatrix3
{
private:

  //---------------------------------------------------------------------//
  //- Private variables                                                 -//
  //---------------------------------------------------------------------//

  std::array<T, 9> m_data;

public:

  //---------------------------------------------------------------------//
  //- Constructor                                                       -//
  //---------------------------------------------------------------------//

  /*!
   * Empty constructor.
   */
  SbgMatrix3(void)
  {
    m_data[0] = static_cast<T>(0.0);
    m_data[1] = static_cast<T>(0.0);
    m_data[2] = static_cast<T>(0.0);
    m_data[3] = static_cast<T>(0.0);
    m_data[4] = static_cast<T>(0.0);
    m_data[5] = static_cast<T>(0.0);
    m_data[6] = static_cast<T>(0.0);
    m_data[7] = static_cast<T>(0.0);
    m_data[8] = static_cast<T>(0.0);
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
    m_data[0] = value00;
    m_data[1] = value01;
    m_data[2] = value02;
    m_data[3] = value10;
    m_data[4] = value11;
    m_data[5] = value12;
    m_data[6] = value20;
    m_data[7] = value21;
    m_data[8] = value22;
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

    m_data[0] = p_raw_data[0];
    m_data[1] = p_raw_data[1];
    m_data[2] = p_raw_data[2];
    m_data[3] = p_raw_data[3];
    m_data[4] = p_raw_data[4];
    m_data[5] = p_raw_data[5];
    m_data[6] = p_raw_data[6];
    m_data[7] = p_raw_data[7];
    m_data[8] = p_raw_data[8];
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

    return m_data[i * 3 + j];
  }

  /*!
   * Get the raw data of the sbgMatrix.
   * 
   * \return                                Raw vector data.
   */
  const T *data(void) const
  {  
    return static_cast<const T*>(m_data.data());
  };
};
}

#endif // SBG_MATRIX_3_H
