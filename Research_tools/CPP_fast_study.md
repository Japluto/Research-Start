# C++ 5日速通 
---
## Day 1 笔记

> 目标：脱离C，开始用C++的方式思考和写代码。理解“类”是C++的灵魂。主攻 PPT 1, 2, 8。

-----

### 一、从 C 到 C++ (C++新特征)

C++ 是C的超集，你所有的C语言知识（循环、判断、函数、指针）在C++中依然有效。但C++引入了许多新特性，让代码更安全、更简洁。

#### 核心知识点

#### 1\. 输入输出流 (iostream) 与命名空间 (namespace)

  - **输入输出流 (`iostream`)**：

      - C++ 使用 `<iostream>` 头文件进行标准输入输出。
      - 抛弃 `printf()` 和 `scanf()`。
      - `cout`：标准输出流对象（对应屏幕）。
      - `cin`：标准输入流对象（对应键盘）。
      - `<<`：**插入运算符**，用于 `cout`。
      - `>>`：**提取运算符**，用于 `cin`。
      - `endl`：一个**操纵符**，用于换行并**刷新缓冲区**（比 `\n` 功能多）。

  - **命名空间 (`namespace`)**：

      - 为什么需要？为了解决“命名冲突”。比如你写的函数叫 `max`，C++标准库里也有个 `max`，编译器就蒙了。
      - 命名空间就是给你的变量/函数划定一个“领地”。
      - `cin` 和 `cout` 都属于 `std` (standard) 领地。
      - **使用方式**：
        1.  **方式一 (推荐)**：在代码开头使用 `using namespace std;`。
        2.  **方式二 (更规范)**：每次使用时都明确指定领地 `std::cout << "Hello";`。
        3.  **方式三 (折中)**：只声明你需要的 `using std::cout; using std::cin;`。

    <!-- end list -->

    ```cpp
    #include <iostream> // 1. 包含C++头文件 (没有.h)

    // 2. 使用 std 命名空间 (方式一)
    using namespace std;

    int main() {
        int my_var;
        
        // 3. 使用 cout 和 cin
        cout << "请输入一个整数：" << endl;
        cin >> my_var;
        cout << "你输入的值是: " << my_var << endl;
        
        return 0;
    }
    ```

#### 2\. 引用 (Reference) - (C中没有，重点)

  - **定义**：引用是 C++ 的一个核心补充，你可以把它理解为变量的“别名”。它不是指针，它不存储地址，它就是那个变量本身。
  - **声明**：引用在声明时**必须**初始化，并且一旦绑定，就不能再更改为其他变量的别名。
    ```cpp
    int a = 10;
    int& b = a; // b 是 a 的别名。b 和 a 现在是同一个东西。

    b = 20;      // 等价于 a = 20;
    cout << a;   // 输出 20
    ```
  - **与指针的对比**：
      - 指针需要用 `&` 获取地址，用 `*` 解引用。
      - 引用在声明时使用 `&`，但使用时**就像普通变量一样**，更简洁、更安全（没有空引用，也不能随意指向其他内存）。
  - **最重要用途**：作为函数参数，实现“引用传递”，完美替代 C 语言中的指针参数。
    ```cpp
    // C 语言风格 (传递地址)
    void swap_c(int* a, int* b) {
        int temp = *a;
        *a = *b;
        *b = temp;
    }

    // C++ 风格 (传递引用)
    void swap_cpp(int& a, int& b) {
        int temp = a;
        a = b;
        b = temp;
    }

    int main() {
        int x = 1, y = 2;
        
        // C 调用
        swap_c(&x, &y);   // 必须取地址
        
        // C++ 调用
        swap_cpp(x, y); // 像普通函数一样调用，非常直观
    }
    ```

#### 3\. 动态内存 (new / delete)

  - **C++ 风格**：抛弃 `malloc` 和 `free`。

  - **`new`**：

    1.  **分配内存**。
    2.  `new` 是**运算符**，不是函数。
    3.  它**自动计算**所需字节数 (不需要 `sizeof`)。
    4.  它**自动返回**正确的类型 (不需要类型转换)。

  - **`delete`**：

    1.  **释放内存**。
    2.  如果你 `new` 的是数组 `[]`，释放时必须用 `delete []`。

  - **规则 (必考)**：

      - `new` 对应 `delete`。
      - `new []` 对应 `delete []`。
      - 绝不能混用（`new` 对应 `free`，或 `malloc` 对应 `delete` 都是灾难）。

    <!-- end list -->

    ```cpp
    // C 语言
    int* p_c = (int*) malloc(sizeof(int));
    free(p_c);
    int* p_arr_c = (int*) malloc(10 * sizeof(int));
    free(p_arr_c);

    // C++
    int* p_cpp = new int;
    delete p_cpp;
    int* p_arr_cpp = new int[10];
    delete [] p_arr_cpp;
    ```

#### 4\. const (替代 \#define)

  - **C 风格**：`#define PI 3.14159`
  - **C++ 推荐**：`const double PI = 3.14159;`
  - **优势**：`const` 是一个**变量**，它有明确的**数据类型**，受编译器类型检查的保护，更安全、更易于调试。`#define` 只是预处理器无脑的**文本替换**。

#### 5\. 函数重载 (Overload)

  - **定义**：C++ 允许在**同一作用域**内，有多个**同名**函数。
  - **规则**：只要它们的**参数列表**不同（参数的**个数**、**类型**、**顺序**不同），编译器就能自动根据你传入的实参来匹配对应的函数。
  - **注意**：函数的**返回值不同**，**不能**构成重载。
    ```cpp
    int max(int a, int b) { ... }
    double max(double a, double b) { ... }
    int max(int a, int b, int c) { ... }

    max(10, 20);       // 调用第一个
    max(3.14, 2.71);   // 调用第二个
    max(1, 2, 3);      // 调用第三个
    ```

#### 6\. C++ 效率技巧 (来自 PPT 8)

  - **前置++**：优先使用 `++i` 而不是 `i++`。
  - **原因**：对于 C 语言 `int` 这种内置类型没区别。但对于C++的**类**（比如迭代器），`i++` (后置) 需要创建一个临时对象来保存 `i` 自增前的值，而 `++i` (前置) 直接自增并返回，效率更高。

#### 实例与练习 (Day 1 - 模块一)

#### 练习1：C++ 版 `swap` 函数

  - **题目**：编写一个程序，要求用户输入两个整数，然后调用一个 `swap` 函数交换它们的值，最后输出交换后的结果。
  - **要求**：
    1.  必须使用 `cin` 和 `cout`。
    2.  `swap` 函数必须使用**引用 (`&`)** 来实现，而不是指针。

-----

#### 练习1参考答案

```cpp
#include <iostream>

// 使用命名空间
using namespace std;

// 1. swap 函数使用引用&
// a 和 b 分别是 main 函数中 x 和 y 的别名
void swap_cpp(int& a, int& b) {
    int temp = a;
    a = b;
    b = temp;
}

int main() {
    int x, y;

    // 2. 使用 cout 和 cin
    cout << "请输入两个整数 x 和 y：" << endl;
    cin >> x >> y;

    cout << "交换前: x = " << x << ", y = " << y << endl;

    // 3. C++风格调用，直接传变量名
    swap_cpp(x, y);

    cout << "交换后: x = " << x << ", y = " << y << endl;

    return 0;
}
```

-----

### 二、类的入门 (C++的灵魂)

这是你从 C 到 C++ 最重要的一步。`class` 是 C++ 的灵魂。

#### 核心知识点

#### 1\. 什么是类 (Class)？

  - **定义**：类 (Class) 是 C 语言 `struct` 的超级升级版。
  - **C 的 `struct`**：只能\*\*“打包”数据\*\*（变量）。
  - **C++ 的 `class`**：既能打包**数据**（称为 **数据成员** 或 **属性**），也能打包**函数**（称为 **成员函数** 或 **方法**）。
  - **C++ 的 `struct` vs `class`**：
      - 在 C++ 中，`struct` 几乎等同于 `class`，它也可以有成员函数。
      - **唯一区别 (考点)**：
          - `struct` 默认的访问权限是 `public`。
          - `class` 默认的访问权限是 `private`。

#### 2\. 封装 (Encapsulation) 与访问权限

  - **定义**：封装是面向对象的第一大特性。它将“数据”和“操作数据的函数”打包在一起，并对外部隐藏（`private`）内部的实现细节，只暴露（`public`）安全的接口。
  - **三个关键字**：
      - `public` (公有)：类的**外部**（如 `main` 函数）和**内部**（成员函数）**都可以**访问。这是类的“接口”。
      - `private` (私有)：**只能**被**本类的成员函数**访问。类的外部（如 `main` 函数）**不能**访问。这是“被保护的内部数据”。
      - `protected` (保护)：和 `private` 类似，但允许“子类”访问。（Day 4 讲继承时详谈）。

#### 3\. 类的定义与实例化

  - **定义类 (图纸)**：

    ```cpp
    // 定义一个 "Student" 类
    class Student {

    private: // 私有数据
        std::string name;
        int id;

    public: // 公有函数 (接口)
        // 成员函数可以直接访问私有数据
        void set_info(std::string n, int i) {
            name = n; // OK
            id = i;   // OK
        }

        void display() {
            cout << "Name: " << name << ", ID: " << id << endl;
        }

    }; // 注意：类定义以分号结尾！
    ```

  - **实例化对象 (实物)**：

    ```cpp
    int main() {
        // s1 是 Student 类的一个对象 (也叫“实例”)
        Student s1; 
        Student s2;
    }
    ```

  - **访问成员**：

      - 使用**成员访问运算符 `.` (点运算符)**。

    <!-- end list -->

    ```cpp
    s1.set_info("Tom", 101); // 正确，set_info() 是 public 的
    s1.display();              // 正确，display() 是 public 的

    s1.name = "Jerry"; // **编译错误！** `name` 是 private 的
    cout << s1.id;     // **编译错误！** `id` 是 private 的
    ```

#### 4\. 工程结构：接口与实现分离 (.h / .cpp)

一个好的 C++ 规范是把类的**声明**和**实现**分开。这就像 C 语言的多文件工程。

  - **`student.h` (头文件 - 接口/声明)**：

      - 告诉外界这个类“能做什么”。

    <!-- end list -->

    ```cpp
    #ifndef STUDENT_H
    #define STUDENT_H

    #include <string>

    class Student {
    private:
        std::string name;
        int id;

    public:
        void set_info(std::string n, int i); // 1. 只做函数声明 (原型)
        void display();                      // 2. 只做函数声明 (原型)
    };

    #endif
    ```

  - **`student.cpp` (源文件 - 实现/定义)**：

      - 具体说明这个类“怎么做”。

    <!-- end list -->

    ```cpp
    #include "student.h"  // 1. 必须包含自己的头文件
    #include <iostream>

    using namespace std;

    // 2. 使用 `::` (作用域解析运算符) 来指明这个函数属于 Student 类
    void Student::set_info(string n, int i) {
        name = n; // 3. 在这里写函数的具体实现
        id = i;
    }

    void Student::display() {
        cout << "Name: " << name << ", ID: " << id << endl;
    }
    ```

  - **`::` (作用域解析运算符)**：

      - C++ 中非常重要的运算符。
      - `Student::display()` 的意思是“`Student` 作用域下的那个 `display` 函数”。
      - `std::cout` 的意思是“`std` 命名空间下的那个 `cout` 对象”。

#### 实例与练习 (Day 1 - 模块二)

#### 练习2：定义 `Box` 类

  - **题目**：定义一个 `Box` 类，用来描述一个箱子。
  - **要求**：
    1.  **数据成员 (私有 `private`)**：
          - `double length;`
          - `double width;`
          - `double height;`
    2.  **成员函数 (公有 `public`)**：
          - `void set_data(double l, double w, double h);` // 用来设置长宽高
          - `double get_volume();` // 用来计算并返回体积 (长 \* 宽 \* 高)
          - `void display();` // 用来在屏幕上输出 "Box volume: [体积]"
  - **提示**：
    1.  在 `get_volume()` 函数中，你可以直接访问 `length`, `width`, `height`。
    2.  在 `main` 函数中，创建一个 `Box` 对象 `b1`，调用 `b1.set_data(...)`，然后调用 `b1.display()`。

-----

#### 练习2参考答案

```cpp
#include <iostream>

using namespace std;

// 1. 定义 Box 类
class Box {
// 2. 定义私有数据成员
private:
    double length;
    double width;
    double height;

// 3. 定义公有成员函数
public:
    // 设置数据的函数
    void set_data(double l, double w, double h) {
        length = l;
        width = w;
        height = h;
    }

    // 计算体积的函数
    double get_volume() {
        return length * width * height;
    }

    // 显示体积的函数
    // 内部调用了另一个成员函数 get_volume()
    void display() {
        cout << "Box volume: " << get_volume() << endl;
    }
};

int main() {
    // 4. 创建 Box 对象 b1
    Box b1;

    // 5. 调用公有函数设置数据
    b1.set_data(10.0, 5.0, 2.0);

    // 6. 调用公有函数显示结果
    b1.display(); // 应该输出 "Box volume: 100"
    
    // 也可以单独获取体积
    double vol = b1.get_volume();
    cout << "The volume is " << vol << endl;

    return 0;
}
```