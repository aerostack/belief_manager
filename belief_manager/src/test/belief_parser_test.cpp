#include "../include/belief_lexer.h"
#include "../include/belief_parser.h"
#include <gtest/gtest.h>

bool nextTokenEquals(BeliefLexer lexer, std::string& text, std::string first, std::string second) {
  auto tkn = lexer.nextToken(text);
  return tkn.first == first && tkn.second == second;
}

TEST(LexerTests, correctTokens0) {
  std::string text = "one,(12.3)test?two";
  BeliefLexer lexer;
  EXPECT_TRUE(nextTokenEquals(lexer, text, "word", "one"));
  EXPECT_TRUE(nextTokenEquals(lexer, text, ",", ""));
  EXPECT_TRUE(nextTokenEquals(lexer, text, "(", ""));
  EXPECT_TRUE(nextTokenEquals(lexer, text, "word", "12.3"));
  EXPECT_TRUE(nextTokenEquals(lexer, text, ")", ""));
  EXPECT_TRUE(nextTokenEquals(lexer, text, "word", "test"));
  EXPECT_TRUE(nextTokenEquals(lexer, text, "var", "?two"));
  EXPECT_TRUE(nextTokenEquals(lexer, text, "$", ""));
}

TEST(ParserTests, correctSyntax0) {
  std::string text = "pred(arg1, (12, ?y, ?))";
  BeliefParser parser;
  auto predicates = parser.parse(text);

  ASSERT_EQ(predicates.size(), 1);

  EXPECT_EQ(predicates[0].name, "pred");
  ASSERT_EQ(predicates[0].args.size(), 2);

  EXPECT_EQ(predicates[0].args[0].name, "arg1");
  EXPECT_EQ(predicates[0].args[0].args.size(), 0);
  EXPECT_EQ(predicates[0].args[1].name, "");
  ASSERT_EQ(predicates[0].args[1].args.size(), 3);

  EXPECT_EQ(predicates[0].args[1].args[0].name, "12");
  EXPECT_EQ(predicates[0].args[1].args[0].args.size(), 0);
  EXPECT_EQ(predicates[0].args[1].args[1].name, "?y");
  EXPECT_EQ(predicates[0].args[1].args[1].args.size(), 0);
  EXPECT_EQ(predicates[0].args[1].args[2].name, "?");
  EXPECT_EQ(predicates[0].args[1].args[2].args.size(), 0);
}

TEST(ParserTests, correctSyntax1) {
  std::string text = "pred((12, ?y, ?), arg2)";
  BeliefParser parser;
  auto predicates = parser.parse(text);

  ASSERT_EQ(predicates.size(), 1);

  EXPECT_EQ(predicates[0].name, "pred");
  ASSERT_EQ(predicates[0].args.size(), 2);

  EXPECT_EQ(predicates[0].args[0].name, "");
  ASSERT_EQ(predicates[0].args[0].args.size(), 3);
  EXPECT_EQ(predicates[0].args[1].name, "arg2");
  EXPECT_EQ(predicates[0].args[1].args.size(), 0);

  EXPECT_EQ(predicates[0].args[0].args[0].name, "12");
  EXPECT_EQ(predicates[0].args[0].args[0].args.size(), 0);
  EXPECT_EQ(predicates[0].args[0].args[1].name, "?y");
  EXPECT_EQ(predicates[0].args[0].args[1].args.size(), 0);
  EXPECT_EQ(predicates[0].args[0].args[2].name, "?");
  EXPECT_EQ(predicates[0].args[0].args[2].args.size(), 0);
}

TEST(ParserTests, correctSyntax2) {
  std::string text = "pred1(a), pred2(a)";
  BeliefParser parser;
  auto predicates = parser.parse(text);

  ASSERT_EQ(predicates.size(), 2);

  EXPECT_EQ(predicates[0].name, "pred1");
  ASSERT_EQ(predicates[0].args.size(), 1);
  EXPECT_EQ(predicates[0].args[0].name, "a");
  EXPECT_EQ(predicates[0].args[0].args.size(), 0);

  EXPECT_EQ(predicates[1].name, "pred2");
  ASSERT_EQ(predicates[1].args.size(), 1);
  EXPECT_EQ(predicates[1].args[0].name, "a");
  EXPECT_EQ(predicates[1].args[0].args.size(), 0);
}

TEST(ParserTests, incorrectSyntax0) {
  // Unbalanced prenthesis
  std::string text = "pred(((12, ?y, ?), arg2)";
  BeliefParser parser;
  auto predicates = parser.parse(text);

  ASSERT_EQ(predicates.size(), 2);
  EXPECT_EQ(predicates[0].name, "$error$");
  EXPECT_EQ(predicates[1].name, "Error: unexpected token '$'");
}

TEST(ParserTests, incorrectSyntax1) {
  // Empty argument
  std::string text = "pred((12, , ?), arg2)";
  BeliefParser parser;
  auto predicates = parser.parse(text);

  ASSERT_EQ(predicates.size(), 2);
  EXPECT_EQ(predicates[0].name, "$error$");
  EXPECT_EQ(predicates[1].name, "Error: unexpected token ','");
}

TEST(ParserTests, incorrectSyntax2) {
  // Empty argument
  std::string text = "pred((12, @, ?), arg2)";
  BeliefParser parser;
  auto predicates = parser.parse(text);

  ASSERT_EQ(predicates.size(), 2);
  EXPECT_EQ(predicates[0].name, "$error$");
  EXPECT_EQ(predicates[1].name, "Error: unexpected symbol '@'");
}

TEST(OtherTests, toString0) {
  BeliefParser::Predicate pred = {.name = "a"};
  BeliefParser parser;
  std::string str = parser.toString(pred);

  EXPECT_EQ(str, "a");
}

TEST(OtherTests, toString1) {
  BeliefParser::Predicate pred = {.name = "a", .args={{.name="2"}, {.name="", .args={{.name="1"}, {.name="2"}}}}};
  BeliefParser parser;
  std::string str = parser.toString(pred);

  EXPECT_EQ(str, "a(2, (1, 2))");
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
