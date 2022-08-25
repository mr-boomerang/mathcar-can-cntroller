#include <functional>
#include <iostream>
#include <utils/params.hpp>
#include <utils/str_utils.h>
//#include <utils/stl_utils.h>

// Click on me, shows demo usage and functionality.
DEFINE_PARAMS_ENUM(Channel, Red, Green, Blue);

DEFINE_PARAMS_ENUM(StereoType, BM, SGBM, GBM);

class check_setter {
public:
  check_setter() {
    val_ = 10;
    sval_ = "tt";
  }
  void setval(int val) { val_ = val; }
  void setstr(std::string sval) { sval_ = sval; }
  int getval() { return val_; }
  std::string getstr() { return sval_; }

private:
  /* data */
  int val_;
  std::string sval_;
};

template <typename T>
typename std::enable_if<params_enum::is_enum<T>::value, bool>::type
print_typ() {
  std::cout << "It's params enum mate." << std::endl;
}

template <typename T>
typename std::enable_if<!params_enum::is_enum<T>::value, bool>::type
print_typ() {
  std::cout << "It's not params enum mate." << std::endl;
}

void print_val(int val) { std::cout << "val = " << val << std::endl; }

void print_val(const double &val) { std::cout << "val = " << val << std::endl; }

bool print_val_cb(const double &val) {
  std::cout << "[callback]: val = " << val << std::endl;
  if (val == 14.5)
    return false;
  else
    return true;
}

int main(int argc, char **argv) {
  try {
    print::init(&argc, &argv);
    std::cout << "Starting..." << std::endl;
#define LOAD_OR_DUMP 1 // Load: 0, Dump: 1
#if LOAD_OR_DUMP

    /************** In code parameter definition and dumping ******************/

    /*---------------Setting and Initing parmaeter ---------------------------*/
    //_s is a custom operator in str_utils.h, which makes text in double qoutes
    // a
    // string object and not const char *.
    // useful when std::string is an argument type with explicit specified, and
    // more often useful for template functions which auto deduce the type, like
    // below "set" function decuces type based on second argument.

    // first interface. param_utils.set(name, value);
    // This interface is available for all types, even custom class and
    // structures. For algorithmic type (see second interface), some properties
    // are auto computed.
    // max = 2*val, min = -2*val, interval = 1(integer), 0.1(floating point)
    //
    // params_utils.set("p0", "It's p0"); //type is const char *.
    // if (!params_utils.set("p0", "It's p0"_s)) // type is std::string, RETURNS
    if (!params_utils.set(
            "p0", std::string("It's p0"))) // type is std::string, RETURNS
      std::cout << "Parameter p0 was set successfully, with type "
                << params_utils.get_type("p0") << std::endl;
    else
      std::cout << "Something went wrong while setting p0" << std::endl;

    int p = 10; // To showcase that param is not linked to the original value
                // setter.
    std::cout << "p1 set success(0 is success) = " << params_utils.set("p1", p)
              << std::endl; // type is int, properties autocomputed.
    std::cout << "has changed = " << std::boolalpha
              << params_utils.has_changed("p1") << std::endl;

    // second interface. param_utils.set(name, value, min, max, interval);
    // This interface is only provided for arithmeting type, i.e. int, char,
    // unsigned char, unsigned int, float, double etc...
    // bool as well(doesn't make lot of sense for bool though).
    // NOTE: Set the interval to 0 if you don't want to disallow certain values.
    params_utils.set("p2", 15, 0, 100, 0);         // type is int
    params_utils.set("p3", 5.5, -30.0, 30.0, 0.0); // type is double
    // params_utils.set("p3", 5.5f, -30.0f, 30.0f, 1.0f); //type is float
    std::cout << std::boolalpha << "callback set successfully? "
              << params_utils.set_validator<double>("p3", &print_val_cb)
              << std::endl;

    /*---------------Getting parmaeter ---------------------------*/
    // Capturing the value of parameter of type int in a variable of type int
    // would require updating the value. Whereas C++ provides us a safe way of
    // storing the location of a variable, i.e. reference. Hence, we capture the
    // parameter in int& or in general <type>&. We also want it as a const
    // parameter so that changing the value is done via set function and not
    // randomly, this allows for validation of value for the parameter. For
    // simplicity we define param_type namespace where basic types are typedef'd
    // to the form we want them in. for e.g. Int, Double, Float, String,
    // UnsignedChar, Char. Notice that the types are typedef'd in Camel Casing
    // and is the intended and recommended format. One can define their own
    // params type if they wish to. for e.g.
    //      DEFINE_PARAMS_TYPE(int, I);
    //      DEFINE_PARAMS_TYPE(MyClass, MyCls);

    // Let's refer to cp0, cp1, cp2, cp3 as captured parameter.
    params_type::Int cp1 = params_utils.get<int>("p1");
    const int &cp2 = params_utils.get<int>(
        "p2"); // this is what params_type::Int is typedef'd to.
    params_type::Double cp3 = params_utils.get<double>("p3"); // double

    /*----------------------Demo----------------------------------------------*/
    // P initialized value of cp1, but cp1 is not linked with p.
    PRINT_SEPERATOR;
    std::cout << "[cp1:: before]: p = " << p << ", cp1 = " << cp1
              << ", min = " << params_utils.get_min<int>("p1")
              << ", max = " << params_utils.get_max<int>("p1")
              << ", interval = " << params_utils.get_interval<int>("p1")
              << ", val = " << cp1 << std::endl;

    // Setting cp1 to an invalid value, since p was initialized with 10 and
    // automatic setting of properties set max at 20 (2*val), 21 is invalid and
    // set function will return false if it's not able to set the value
    // successfully.
    if (!params_utils.set("p1", 21)) {
      std::cout << "Couldn't set 21 as value for p1." << std::endl;
    }
    params_utils.set("p1", 20); // min and max are inclusive. successful
                                // setting.
    p = 15;
    std::cout << "[cp1:: after]: p = " << p << ", cp1 = " << cp1
              << ", min = " << params_utils.get_min<int>("p1")
              << ", max = " << params_utils.get_max<int>("p1")
              << ", interval = " << params_utils.get_interval<int>("p1")
              << ", val = " << cp1 << std::endl;

    // cp1 = 50; //Uncommenting this line should result in compilation error.
    // cp1 = 10; //Uncommenting this line should result in compilation error.

    PRINT_SEPERATOR;
    // Change properties of an algorithmic type parameter. This is also done via
    //"set" function, as we want to check relational validity of properties,
    // i.e. min < max, min < val < max, interval < max. Providing a seperate
    // interface can potentially betray the fidelity of these relationships.
    std::cout << "[cp2:: before]: min = " << params_utils.get_min<int>("p2")
              << ", max = " << params_utils.get_max<int>("p2")
              << ", interval = " << params_utils.get_interval<int>("p2")
              << ", val = " << cp2 << std::endl;
    // If only properties need updating and not the value pass the current value
    // as the second parameter. if min max and interval are not right for the
    // current value function will return false.
    params_utils.set("p2", cp2, -100, 100, 2);
    std::cout << "[cp2:: after]: min = " << params_utils.get_min<int>("p2")
              << ", max = " << params_utils.get_max<int>("p2")
              << ", interval = " << params_utils.get_interval<int>("p2")
              << ", val = " << cp2 << std::endl;

    PRINT_SEPERATOR;
    params_type::String cp0 =
        params_utils.get<std::string>("p0"); // std::string
    std::cout << "[cp0:: before]: " << cp0 << std::endl;
    // If param type is to be std::string '_s' is important in second arg.
    params_utils.set("p0", "p0 changed"_s);
    std::cout << "[cp0:: after(ref)]: " << cp0 << std::endl;

    std::cout << "[double] cp3: " << cp3 << std::endl;

    // pass it to functions. A captured param of type int can be passed as is to
    // argument type int, const int, const int&. but not int&, this may require
    // declaration of extra variable. It's extra bit of work but ow design
    // leaves
    // the code open to lot of bugs and mistakes.
    // It's a good practice to have reference as const if it's an input.
    // A non-const reference is useful when we want the called function to
    // change
    // the value of variable, but we don't expect that to happen for parameters
    // and we want the change of value to happen by the function "set", hence
    // passing a captured parameter as <type>& might be a good idea in very
    // limited and specific cases.
    PRINT_SEPERATOR;
    print_val(cp1); // int version.
    print_val(cp3); // double version.

    // Setting up the scenario for set parameter to fail due to interval being
    // set.
    PRINT_SEPERATOR;
    params_utils.set("i_inter", 20, 13, 69, 5);
    params_type::Int i_inter = params_utils.get<int>("i_inter");
    std::cout << "[before]: i_inter = " << i_inter << std::endl;
    std::cout << std::boolalpha << "Success during setting i_inter as 34: "
              << params_utils.set("i_inter", 34) << std::endl;
    std::cout << "[after 34]: i_inter = " << i_inter << std::endl;
    std::cout << std::boolalpha << "Success during setting i_inter as 62: "
              << params_utils.set("i_inter", 62) << std::endl;
    std::cout << "[after 62]: i_inter = " << i_inter << std::endl;
    std::cout << std::boolalpha << "Success during setting i_inter as 18: "
              << params_utils.set("i_inter", 18) << std::endl;
    std::cout << "[after 18]: i_inter = " << i_inter << std::endl;

    params_utils.set("f_inter", 1.5f, 0.0f, 10.0f, 0.75f);
    params_type::Float f_inter = params_utils.get<float>("f_inter");
    std::cout << "[before]: f_inter = " << f_inter << std::endl;
    std::cout << std::boolalpha << "Success during setting f_inter as 5.2: "
              << params_utils.set("f_inter", 5.2f) << std::endl;
    std::cout << "[after 5.2]: f_inter = " << f_inter << std::endl;
    std::cout << std::boolalpha << "Success during setting f_inter as 6.1: "
              << params_utils.set("f_inter", 6.1f) << std::endl;
    std::cout << "[after 6.1]: f_inter = " << f_inter << std::endl;
    std::cout << std::boolalpha << "Success during setting f_inter as 3.0: "
              << params_utils.set("f_inter", 3.0f) << std::endl;
    std::cout << "[after 3.0]: f_inter = " << f_inter << std::endl;

    PRINT_SEPERATOR;
    std::cout << "Printing value just by changing it as print_val function is "
                 "a callback"
              << std::endl;
    std::cout << "[before 13.2] value in const ref: " << cp3 << std::endl;
    params_utils.set("p3", 13.2);
    std::cout << "[after 13.2] value in const ref: " << cp3 << std::endl;
    std::cout << "print_val_cb, return false if value being set is 14.5, jlt. "
              << "Now, when we try to set 14.5 it will return false and "
              << "param value won't change" << std::endl;
    std::cout << "[before 14.5] value in const ref: " << cp3 << std::endl;
    params_utils.set("p3", 14.5);
    std::cout << "[after 14.5] value in const ref: " << cp3 << std::endl;

#define ENUM_DEMO 1
#ifdef ENUM_DEMO
    PRINT_SEPERATOR;
    std::cout << "Initing enum.." << std::endl;
    params_enum::Channel c = params_enum::Channel::Green;
    std::cout << "Channel = " << c << std::endl;
    // PENS = params_enum. PENS is a macro defined in params_enum.
    c = PENS::Channel::from_string("Red");
    std::cout << "Channel changed to " << c << " using from_string method"
              << std::endl;
    if (c == PENS::Channel::Red) {
      std::cout << "c is equal to Red" << std::endl;
    }
    params_utils.set("color_channel", c);
#if 0
    std::cout << params_utils.get_type("color_channel") << std::endl;
    std::cout << "is_params_enum = " << params_enum::is_enum<params_enum::Channel>::value << std::endl;
    std::cout << "is_params_enum = " << params_enum::is_enum<std::string>::value << std::endl;
    print_typ<params_enum::Channel>();
    print_typ<std::string>();
    print_typ<int>();
#else
    PRINT_SEPERATOR;
    const params_type::Channel &cp =
        params_utils.get<PENS::Channel>("color_channel");
    std::cout << "cp_enum[before] = " << cp << std::endl;
    params_utils.set("color_channel",
                     params_enum::Channel(PENS::Channel::Green));
    std::cout << "cp_enum[after] = " << cp << std::endl;
    PENS::Channel cpv1 = params_utils.get_val("color_channel", c);
    std::cout << "cp_enum_val1[] = " << cpv1 << std::endl; // Should print
                                                           // Green.
    std::cout << "Demo: param not found with color_chne whereas it should be "
                 "color_channel."
              << std::endl;
    PENS::Channel cpv2 =
        params_utils.get_val("color_chne", c);             // con contains Red.
    std::cout << "cp_enum_val2[] = " << cpv2 << std::endl; // should print Red.
    std::cout << "All types of channel: " << std::endl;
    vec_str tn = PENS::Channel::all_values_names();
    std::cout << utils::join_stl(tn) << std::endl;
    PENS::Channel::all_values_stream(std::cout, ";");
    std::cout << std::endl;
    switch (cpv2) {
    case PENS::Channel::Red:
      std::cout << "It's red" << std::endl;
      break;
    case PENS::Channel::Green:
      std::cout << "It's green" << std::endl;
      break;
    case PENS::Channel::Blue:
      std::cout << "It's blue" << std::endl;
      break;
    default:
      std::cout << "It's default." << std::endl;
      break;
    }
#endif
#endif

#define LINK 1
#if LINK
    PRINT_SEPERATOR;
    int lv = 10;
    std::cout << "lv = " << lv << ", p2 = " << params_utils.get_val("p2", 12)
              << std::endl;
    params_utils.link("p2", &lv);
    std::cout << "lv = " << lv << ", p2 = " << params_utils.get_val("p2", 12)
              << std::endl;
    params_utils.set("p2", 18);
    std::cout << "lv = " << lv << ", p2 = " << params_utils.get_val("p2", 12)
              << std::endl;
#endif

    //test case for params when ostream operator<< doesn't exist.
    check_setter cscs;
    params_utils.set("check_setter_dump", cscs);

    // Can dump all types with << operator defined for ostream.
    params_utils.dump("default.cfg");
#else
    /****************** Load parameter from cfg file and list *****************/
    std::cout << "loading.." << std::endl;
    // Only basic type supported i.e. std::string, bool, int, float... refer to
    // comment before function definition to get the full list
    params_utils.load("default.cfg");
    std::cout << "getting list.." << std::endl;
    // std::string lst = params_utils.list_names(); //just param names
    std::string lst =
        params_utils.list_full("\n"); // params with vales and properties.
    std::cout << lst;
    std::cout << "\nDone." << std::endl;
#endif

#define ROS_NODE                                                               \
  0 // keep this zero, below code is just for demo, with current cmakelists it
    // won't compile
#if ROS_NODE
    /*-------------------- Integration with ros ------------------------------*/
    /*----- Server end ------*/
    ros::init(argc, argv, "<node name>");

    // call following fuction (after ros::init and before you expect service to
    // be used) to start two Ros Services:
    // 1. /<node name>/params_server/list
    // 2. /<node name>/params_server/set
    // If utils is compiled without ros then following function won't exist and
    // should cause an compilation error. If ros exists but the code is
    // executed outside the perview of ros, this function call should return
    // false. Failure to start services within preview of ros will print
    // exception message and return false. true is returned on successful
    // initialization of services.
    if (params_utils.start_ros_service())
      std::cout << "Ros services for parameters started successfully."
                << std::endl;
    // NOTE: Parameters can be set and captured before ros service is started,
    // not calling this function should result in ServiceClient to fail

    /*------ Client end -------*/
    // Client end is not something which is going to be useful but jic.
    ros::ServiceClient param_list_client =
        n.serviceClient<utils::ParamList>("/<node name>/params_server/list");
    utils::ParamList lst_srv;
    lst_srv.request.list_full = true; // to get info from list_full function.
    param_list_client.call(lst_srv);  // returns true on success.
    std::string lst = lst_srv.list;   // A "\n" seperated list of params.

    ros::ServiceClient param_set_client =
        n.serviceClient<utils::ParamSet>("/<node name>/params_server/set");
    utils::ParamSet set_srv;
    set_srv.request.name = "p1";    // to get info from list_full function.
    set_srv.request.value = "12";   // to get info from list_full function.
    param_set_client.call(set_srv); // returns true on success.
    bool param_set = set_srv.success;
#endif

#define SETTER 1
#if SETTER
    PRINT_SEPERATOR;
    params_utils.set("setter1", 15);
    params_utils.set("setter_s", STR("temp"));
    check_setter cs;
    ParamSetter<void, int> f =
        std::bind(&check_setter::setval, &cs, std::placeholders::_1);
    params_utils.add_setter("setter1", f);
    std::cout << "Before setter = " << params_utils.get_val("setter1", 0)
              << ", " << cs.getval() << std::endl;
    params_utils.set("setter1", 20);
    std::cout << "After setter = " << params_utils.get_val("setter1", 0) << ", "
              << cs.getval() << std::endl;

    params_utils.add_setter("setter_s", &check_setter::setstr, &cs);
    std::cout << "Before setter = "
              << params_utils.get_val("setter_s", STR("0")) << ", "
              << cs.getstr() << std::endl;
    params_utils.set("setter_s", STR("ppap"));
    std::cout << "After setter = " << params_utils.get_val("setter_s", STR("0"))
              << ", " << cs.getstr() << std::endl;
    PRINT_SEPERATOR;
#endif

#define VECTOR 1
#if VECTOR
    std::vector<int> vec;
    vec.push_back(10);
    vec.push_back(20);
    vec.push_back(30);
    params_utils.set("vec_param", vec);
    std::cout << "vec_type = "
              << utils::demangle_type(params_utils.get_type("vec_param"))
              << std::endl;
    const std::vector<int> &vec_ref =
        params_utils.get<std::vector<int>>("vec_param");
    std::cout << "vec[0] = " << vec_ref[0] << std::endl;
#endif

  }
  // For cases where user has commited a mistake, like capturing the parameter
  // for the wrong type or trying to capture variable which hasn't been set yet
  // an exception is thrown of ParamExcept, e.what() will print the error string
  catch (ParamExcept e) {
    std::cout << "Param exception!!! " << e.what() << std::endl;
  }

  return 0;
}
