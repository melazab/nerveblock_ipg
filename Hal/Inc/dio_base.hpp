#ifndef __DIO_BASE_H
#define __DIO_BASE_H

#include <cstdint>    // for std::uint32_t



class dio_base
{
public:
    typedef enum
    {
        LOW  = 0,
        HIGH = 1
    } dioState_t;

    enum logic_level
    {
        Positive_Logic,
        Negative_Logic,
    };

    typedef std::uint32_t dioPort_t;

    typedef std::uint32_t dioPin_t;

    // Constructors and Destructors
    dio_base()          = default;
    virtual ~dio_base() = default;

    // Class methods
    virtual void write( dioState_t state ) = 0;
    virtual dioState_t read()              = 0;

    /// @brief Set the pin to its activated logic level.
    /// @details Dependent on the logic level setting of the object.
    ///          HIGH for Positive_Logic.
    ///          LOW for Negative_logic.
    ///          Equivalent to On, Enable, Assert.
    virtual void activate() = 0;

    /// @brief Set the pin to its deactivated logic level.
    /// @details Dependent on the logic level setting of the object.
    ///          LOW for Positive_Logic.
    ///          HIGH for Negative_logic.
    ///          Equivalent to Off, Disanable, Deassert.
    virtual void deactivate() = 0;

protected:
    dioPort_t port;
    dioPin_t pin;
    dioState_t pin_level_activated;
    dioState_t pin_level_deactivated;

private:
    // Delete the copy constructor and assignment operator.
    // This prevents the compiler from generating them.
    // We don't want to allow copying of dio objects.
    dio_base( const dio_base& other )      = delete;
    dio_base& operator=( const dio_base& ) = delete;

    // Delete the move constructor and assignment operator.
    // This prevents the compiler from generating them.
    // We don't want to allow moving of dio objects.
    dio_base( dio_base&& other )            = delete;
    dio_base& operator=( dio_base&& other ) = delete;
};

#endif /* __DIO_BASE_HPP */
