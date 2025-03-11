#ifndef MSGGROUP_HPP
#define MSGGROUP_HPP

class MsgGroup {
public:
    // 构造函数
    MsgGroup(int value);

    // 析构函数
    ~MsgGroup();

    // 公共成员函数
    void setValue(int value);
    int getValue() const;

private:
    // 私有成员变量
    int m_value;

    // 私有成员函数
    void printValue() const; // 新增的私有函数
};

#endif // MSGGROUP_HPP