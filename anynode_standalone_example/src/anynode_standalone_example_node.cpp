/*!
 * @file     anynode_standalone_example_node.cpp
 * @author   Dario Bellicoso
 * @date     Sep 21, 2015
 * @brief
 */

#include "anynode_standalone_example/AnyNodeStandaloneExample.hpp"
#include <any_node/any_node.hpp>
#include <ros/ros.h>

int main(int argc, char **argv) {
  any_node::Nodewrap<anynode_standalone_example::AnyNodeStandaloneExample> node(
      argc, argv, "anynode_standalone_example", 1);
  node.execute();
  return 0;
}
