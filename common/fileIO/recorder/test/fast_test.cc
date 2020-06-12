#include <sqlite_modern_cpp.h>
#include <iostream>
using namespace sqlite;
using namespace std;

int main() {
  try {
    // creates a database file 'dbfile.db' if it does not exists.
    database db("dbfile.db");

    // executes the query and creates a 'user' table
    db << "create table if not exists user ("
          "   _id integer primary key autoincrement not null,"
          "   age int,"
          "   name text,"
          "   weight real"
          ");";
    db << "PRAGMA synchronous = OFF";
    // inserts a new user record.
    // binds the fields to '?' .
    // note that only types allowed for bindings are :
    //      int ,long, long long, float, double
    //      string , u16string
    // sqlite3 only supports utf8 and utf16 strings, you should use
    // std::string for utf8 and std::u16string for utf16. note that u"my
    // text" is a utf16 string literal of type char16_t * .
    while (1)
      db << "insert into user (age,name,weight) values (?,?,?);" << 20 << u"bob"
         << 83.25;

  } catch (exception& e) {
    cout << e.what() << endl;
  }
}