#include "../include/MyClass.h"

MyClass::MyClass() 
{
    std::cout << "Created an instance of MyClass" << std::endl;
}

MyClass::MyClass(std::string name) 
{   
    m_name = name;
    std::cout << "Created an instance of MyClass with name: " << name << std::endl;
}

std::string MyClass::GetName() const 
{
    return m_name;
}