// Copyright 2011-2018 The Mathworks, Inc.
/**
 * @file can_yaml_parser.cpp
 * @brief yaml parser for can status and control messages
 * @author Divyanshu Goel
 * @date 2017-02-06 (yyyy-mm-dd)
 */
#include <can_driver/can_yaml_parser.h>

#include <string>
#include <vector>

// close the file handle
int CanYamlParser::close_file()
{
  myfile.close();
  return 0;
}
// prints unsigned int vector
void CanYamlParser::print_u_int_vector(
    std::vector<unsigned int> messages_array_temp)
{
  int length = messages_array_temp.size();
  for (int element_no = 0; element_no < length; element_no++)
  {
    if (element_no < length - 1)
    {
      printf("%d, ", messages_array_temp[element_no]);
    }
    else
    {
      printf("%d ", messages_array_temp[element_no]);
    }
  }
  printf("\n");
}
// prints std string vector
void CanYamlParser::print_names_vector(
    std::vector<std::string> messages_array_temp)
{
  int length = messages_array_temp.size();
  for (int element_no = 0; element_no < length; element_no++)
  {
    if (element_no < length - 1)
    {
      printf("%s, ", messages_array_temp[element_no].c_str());
    }
    else
    {
      printf("%s", messages_array_temp[element_no].c_str());
    }
  }
}
// prints dictionary map which contains keya and value that is controls for the
// command exerted
void CanYamlParser::print_dictionary_map(DictionaryMap messages_array_temp)
{
  int length = messages_array_temp.key.size();
  printf("{");
  for (int element_no = 0; element_no < length; element_no++)
  {
    if (messages_array_temp.key[element_no].compare("Max_value") == 0)
    {
      std::cout << "\'" << messages_array_temp.key[element_no]
                << "\' : " << messages_array_temp.value[element_no];
    }
    else
    {
      std::cout << "\'" << messages_array_temp.key[element_no] << "\' : 0b"
                << std::bitset<5>(messages_array_temp.value[element_no]);
    }
    if (element_no != length - 1)
    {
      printf(", ");
    }
  }
  printf("}");
}
// prints a collection of DictionaryMap
void CanYamlParser::print_dictionary_vector(
    std::vector<DictionaryMap> messages_array_temp)
{
  int length = messages_array_temp.size();
  printf("[ ");
  for (int element_no = 0; element_no < length; element_no++)
  {
    print_dictionary_map(messages_array_temp[element_no]);
    if (element_no != length - 1)
    {
      printf(", ");
    }
  }
  printf(" ]");
}
// converts sample string to unsigned int array for the purpose of separating
// parameter_value to a array
void CanYamlParser::get_array(std::string *sample,
                              std::vector<unsigned int> *messages_array)
{
  trim(sample, '[');
  trim(sample, ']');
  trim(sample, ' ');
  int char_location = 0;
  int start_index = 0;
  int length = sample->size();
  std::string number_container;
  if (length == 0)
  {
    char_location = 1;
    messages_array->push_back(-1);
  }
  while (char_location <= length)
  {
    if ((*sample)[char_location] == ',')
    {
      number_container = sample->substr(start_index, char_location);
      trim(&number_container, ',');
      trim(&number_container, ' ');
      start_index = char_location;
      if (number_container.compare("") == 0)
      {
        messages_array->push_back(-1);
      }
      else
      {
        int data = std::stoul(number_container, nullptr, 10);
        messages_array->push_back(data);
      }
    }
    if (char_location == length)
    {
      number_container = sample->substr(start_index, char_location);
      trim(&number_container, ',');
      trim(&number_container, ' ');
      start_index = char_location;
      if (number_container.compare("") == 0)
      {
        messages_array->push_back(-1);
      }
      else
      {
        int data = std::stoul(number_container, nullptr, 10);
        messages_array->push_back(data);
      }
    }
    char_location = char_location + 1;
  }
}
// removes given special character from the given string
int CanYamlParser::remove_special_chars(std::string *sample)
{
  int length = sample->size();
  if (length == 0) return (0);
  int member_count = 1;
  // loop to weed out the special characters
  for (int char_count = 0; char_count < length; char_count++)
  {
    char char_at = (*sample)[char_count];

    if (char_at == '"')
    {
      (*sample)[char_count] = ' ';
    }
    if (char_at == '{')
    {
      (*sample)[char_count] = ' ';
    }
    if (char_at == '}')
    {
      (*sample)[char_count] = ' ';
    }
    if (char_at == '[')
    {
      (*sample)[char_count] = ' ';
    }
    if (char_at == ']')
    {
      (*sample)[char_count] = ' ';
    }
    if (char_at == ',')
    {
      member_count++;
    }
  }
  trim(sample, ' ');
  return (member_count);
}
// function to get names from a string
void CanYamlParser::get_names_array(
    std::string *sample, std::vector<std::string> *messages_array_data)
{
  int arg_no = remove_special_chars(sample);
  if (arg_no == 1)
  {
    messages_array_data->push_back(*sample);
  }
  else
  {
    int length = sample->size();
    int start_index = 0;
    for (int char_count = 0; char_count < length; char_count++)
    {
      char char_at = (*sample)[char_count];
      std::string temp;
      if (char_at == ',')
      {
        temp = sample->substr(start_index, char_count - start_index);
        start_index = char_count + 1;
        trim(&temp, ' ');
        messages_array_data->push_back(temp);
      }
      if (char_count == length - 1)
      {
        temp = sample->substr(start_index, char_count - start_index + 1);
        start_index = char_count + 1;
        trim(&temp, ' ');
        messages_array_data->push_back(temp);
      }
    }
  }
}
// function to get a key and value pair for the dictionary
void CanYamlParser::fetch_key_value_pair(std::string *sample, std::string *key,
                                         unsigned int *value)
{
  trim(sample, ',');
  trim(sample, ' ');
  int delimiter_loc = sample->find(':');
  std::string key_substring = sample->substr(0, delimiter_loc);
  trim(&key_substring, ' ');
  std::string value_substring = sample->substr(delimiter_loc + 1);
  trim(&value_substring, ' ');
  int base = 10;
  delimiter_loc = value_substring.find("0b");
  if (delimiter_loc == 0)
  {
    value_substring = value_substring.substr(2);
    base = 2;
  }
  delimiter_loc = value_substring.find("0x");
  if (delimiter_loc == 0)
  {
    value_substring = value_substring.substr(2);
    base = 16;
  }
  *key = key_substring;
  *value = std::stoul(value_substring, nullptr, base);
}
DictionaryMap CanYamlParser::get_dictionary_map(std::string *sample)
{
  trim(sample, '[');
  trim(sample, ']');
  remove_special_chars(sample);
  trim(sample, ',');
  trim(sample, ' ');
  DictionaryMap dict_temp;
  unsigned int values;
  std::string key;
  int length = sample->size();
  int start_index = 0;
  for (int char_count = 0; char_count < length; char_count++)
  {
    char char_at = (*sample)[char_count];
    std::string temp;
    if (char_at == ',')
    {
      temp = sample->substr(start_index, char_count - start_index);
      fetch_key_value_pair(&temp, &key, &values);
      dict_temp.key.push_back(key);
      dict_temp.value.push_back(values);
      start_index = char_count;
    }
    if (char_count == length - 1)
    {
      temp = sample->substr(start_index, char_count - start_index + 1);
      fetch_key_value_pair(&temp, &key, &values);
      dict_temp.key.push_back(key);
      dict_temp.value.push_back(values);
      start_index = char_count;
    }
  }
  return (dict_temp);
}
// obtains an array of dictionary for a given message id
void CanYamlParser::get_dictionary_array(
    std::string *sample, std::vector<DictionaryMap> *messages_array)
{
  trim(sample, '[');
  trim(sample, ']');
  int length = sample->size();
  int start_index = 0;
  for (int char_count = 0; char_count < length; char_count++)
  {
    std::string substring_temp;
    char char_at = (*sample)[char_count];
    if (char_at == '}')
    {
      substring_temp = sample->substr(start_index, char_count - start_index);
      start_index = char_count;
      messages_array->push_back(get_dictionary_map(&substring_temp));
    }
  }
}
// prints all parameters
void CanYamlParser::param_print_all()
{
  int len = can_io_message.size();
  int element_counter = 0;
  printf("******************\n");
  while (element_counter < len)
  {
    printf("Message Name : %s\n",
           can_io_message[element_counter].message_name.c_str());
    printf("Message ID : 0x%03x\n", can_io_message[element_counter].message_id);
    printf("Message Count : %i\n",
           can_io_message[element_counter].message_count);
    printf("Message Total Length : %i\n",
           can_io_message[element_counter].total_length);
    printf("Messages Start Bits : ");
    print_u_int_vector(can_io_message[element_counter].start_bit);
    printf("Messages Lengths : ");
    print_u_int_vector(can_io_message[element_counter].message_length);
    printf("Messages Validity Bits : ");
    print_u_int_vector(can_io_message[element_counter].validity_bit);
    printf("Message Names : ");
    print_names_vector(can_io_message[element_counter].names);
    printf("\nMessages Dictionary Map : ");
    print_dictionary_vector(can_io_message[element_counter].dictionary);
    printf("\n*****************\n");
    element_counter++;
  }
}
// update the parameter
void CanYamlParser::add_param(std::string *parameter_name,
                              std::string *parameter_value)
{
  if (parameter_name->compare("message_id") == 0)
  {
    can_io_message[vector_count].message_id =
        std::stoul((*parameter_value), nullptr, 16);
  }
  if (parameter_name->compare("message_count") == 0)
  {
    can_io_message[vector_count].message_count =
        std::stoul((*parameter_value), nullptr, 10);
  }
  if (parameter_name->compare("total_length") == 0)
  {
    can_io_message[vector_count].total_length =
        std::stoul((*parameter_value), nullptr, 10);
  }
  if (parameter_name->compare("start_bit") == 0)
  {
    get_array(parameter_value, &can_io_message[vector_count].start_bit);
  }
  if (parameter_name->compare("message_length") == 0)
  {
    get_array(parameter_value, &can_io_message[vector_count].message_length);
  }
  if (parameter_name->compare("validity_bit") == 0)
  {
    get_array(parameter_value, &can_io_message[vector_count].validity_bit);
  }
  if (parameter_name->compare("name") == 0)
  {
    get_names_array(parameter_value, &can_io_message[vector_count].names);
  }
  if (parameter_name->compare("dictionary_map") == 0)
  {
    get_dictionary_array(parameter_value,
                         &can_io_message[vector_count].dictionary);
  }
}
// read file to get dictionaries
void CanYamlParser::read_file(const std::string filepath)
{
  std::string line;
  myfile.open(filepath);
  if (myfile.is_open())
  {
    /* ok, proceed with output */
    while (std::getline(myfile, line))
    {
      char first_char;
      first_char = line[0];
      if (first_char == ' ')
      {
        /// Adding to existing message goes here
        std::string parameter_name = line.substr(0, line.find(':'));
        trim(&parameter_name, ' ');
        std::string parameter_value =
            line.substr(line.find(':') + 1, line.size());
        trim(&parameter_value, ' ');
        add_param(&parameter_name, &parameter_value);
      }
      else
      {
        if (first_char == '\0')
        {
          /// Line End condition
        }
        else
        {
          if (first_char != ' ')
          {
            /// New message Condition
            trim(&line, ':');
            CanMessage temp_message;
            can_io_message.push_back(temp_message);
            vector_count = vector_count + 1;
            can_io_message[vector_count].message_name = line;
          }
        }
      }
    }
  }
  else
  {
    PE$ ct_red("Opening file failed. Please ensure that there is a yaml file")
        pendl;
  }
}
