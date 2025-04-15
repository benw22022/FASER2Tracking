#pragma once
#include <iostream>
#include <string>

class MyClass
{
    public:
        MyClass();
        MyClass(std::string name);

        std::string m_name{""};

        std::string GetName() const;
};