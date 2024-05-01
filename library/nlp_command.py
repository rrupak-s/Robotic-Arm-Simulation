def check_sentence(sentence, words_to_check):
    if sentence is None or not isinstance(sentence, str):
        return -1  # Return -1 for invalid input

    for idx, word in enumerate(words_to_check):
        if word in sentence.lower():
            return idx

    return -1  # Return -1 if none of the words are found

# List of words to check
# words_to_check = ["sprite", "water", "coffee", "orange"]

# Input sentence
# input_sentence = input("Enter a sentence: ")

# result = check_sentence(input_sentence.lower(), words_to_check)

# if result != -1:
#     print(f"The sentence contains '{words_to_check[result]}', and its index is {result}.")
# else:
#     print("The sentence does not contain any of the specified words.")
