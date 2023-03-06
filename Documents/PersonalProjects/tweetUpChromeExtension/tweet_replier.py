import openai
import tweepy
import os
import json
from dotenv import load_dotenv
from conversation_retriever import retrieve_conversation
load_dotenv()  # Load environment variables from .env file

openai.api_key = os.getenv('OPENAI_API_KEY')

def get_reply_suggestions(original_tweet,extra_context="Be witty and smart", desired_suggestion_count=3):

    completion = openai.ChatCompletion.create(
    model="gpt-3.5-turbo",
    messages=[
      {"role": "user", "content": f"Based on this tweet {original_tweet}. \n Come up with {desired_suggestion_count} thoughtful suggested replies. {extra_context}. Suggestion #x. Leave only 1 new line in between suggestions"}
    ]
  )

    formatted_completion = completion.choices[0].message.content
    formatted_completion = formatted_completion.split("\n")

    formatted_completion = list(filter(lambda x: x != '', formatted_completion))
    return formatted_completion

def pretty_print_suggestions(original_tweet, raw_suggestions):
    print(f'Original Tweet: {original_tweet}')
    print('---Suggestions---')
    for i in raw_suggestions:
      print(i)

# original_tweet = "I am loving GPT-3.5 Turbo. It's so much faster"
# original_tweet = retrieve_conversation("1631281412761427969")
# reply_suggestions = get_reply_suggestions(original_tweet,5)
# pretty_print_suggestions(original_tweet, reply_suggestions)

def main_reply_func (tweetID,extra_context="Be witty and smart", desired_suggestion_count=3):
    original_tweet = retrieve_conversation(tweetID)
    reply_suggestions = get_reply_suggestions(original_tweet,extra_context,desired_suggestion_count)
    pretty_print_suggestions(original_tweet, reply_suggestions)
    return reply_suggestions
