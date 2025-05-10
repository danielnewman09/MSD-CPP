/*
We want to be able to create objects like so:

enum class SelectOp
{
    Equal,
    NotEqual,
    GreaterThan,
    LessThan,
    GreaterThanOrEqual,
    LessThanOrEqual
};

enum class SelectType
{
    String,
    Int,
    Double,
    Float,
    Bool
};

enum class KeyType
{
    Primary,
    Foreign
};




struct SelectCondition
{
    std::string column;
    SelectOp op;
    std::string value;
};

class DatabaseTable
{
public:
    DatabaseObject() = default;

    table.select(std::vector<std::string>{"column1", "column2"},  )

    select(std::vector<std::string>)

    bool select();
    bool insert();
    bool update();
    bool remove();

private:
    // The name of the database table
    std::string name_;

    
};

class Child : public DatabaseObject<ChildDatabaseTable>


class Parent 
{
public:


    friend DBModel;

    Parent() = default;

    void setX(int x) { xInt = x; }
    void setY(double y) { yDouble = y; }
    void setZ(string z) { zString = z; }
    int getX() const { return xInt; }
    double getY() const { return yDouble; }
    string getZ() const { return zString; }

    Child getChild() const { return child_; }
    void setChild(Child child) { child_ = child; }

    std::vector<Child> getChildren() const { return children_; }
    void setChildren(std::vector<Child> children) { children_ = children; }

private:
    int xInt;
    double yDouble;
    string zString;

    Child child_;

    std::vector<Child> children_;
};

struct DBEntry
{

};



/*!
 * @brief A row in the database table.
 */
struct DBRow
{
    // 

};

/*!
 * @brief A relationship between two database tables.
 */
struct DBRelationship
{

};

template <typename T>
struct DBColumnMetadata
{
    // The name of the column
    std::string name;


    std::vector<DBRelationship> relationships;
};

struct DBObjectAccessor
{

};

template <typename T>
class DatabaseObject
{
    public:

       
    private:
        // The objects that accommodate reading and writing
        // to the database table
        std::vector<DBObjectAccessor<T>> accessors_;

};

class MyObject
{
    friend class MyObjectTransferObject;

    public:
        MyObject() : xInt(0), yDouble(0.0), zString(""),myBool{false} {}

        int getXInt() const { return xInt; }
        void setXInt(int value) { xInt = value; }
        double getYDouble() const { return yDouble; }
        void setYDouble(double value) { yDouble = value; }
        std::string getZString() const { return zString; }
        void setZString(const std::string& value) { zString = value; }
        bool getMyBool() const { return myBool; }
        void setMyBool(bool value) { myBool = value; }

    private:
       int xInt;
       double yDouble;
       std::string zString;
       bool myBool;
};