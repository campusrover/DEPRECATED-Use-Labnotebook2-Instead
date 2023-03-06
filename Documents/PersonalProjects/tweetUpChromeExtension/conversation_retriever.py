import tweepy
import os
import json
from dotenv import load_dotenv

load_dotenv()  # Load environment variables from .env file

consumer_key=os.getenv('Consumer_API_Key')
consumer_secret = os.getenv('Consumer_API_Secret')
access_token  = os.getenv('Access_Token')
access_token_secret  = os.getenv('Access_Token_Secret')

def get_thread(api, tweet_id):
    # Get the tweet object for the specified ID
    tweet = api.get_status(tweet_id, tweet_mode='extended')
    
    # Initialize an empty list to store the thread
    thread = []
    
    # Keep looping until we've reached the beginning of the thread
    while True:
        # Add the current tweet to the beginning of the thread list
        thread.insert(0, tweet)
        
        # Check if the tweet is a reply to another tweet
        if tweet.in_reply_to_status_id is None:
            # If not, we've reached the beginning of the thread
            break
        
        # Get the tweet object for the tweet being replied to
        tweet = api.get_status(tweet.in_reply_to_status_id, tweet_mode='extended')
    
    return thread

# # Authenticate with Twitter API using your own credentials
auth = tweepy.OAuth1UserHandler(
    consumer_key, consumer_secret,access_token,access_token_secret
)


# Initialize the API client
api = tweepy.API(auth)

# Get the thread for a specific tweet ID
tweet_id = "1631281412761427969"

# Print the text of each tweet in the thread
# iter = 1
# for tweet in thread:
#     print(f'{iter}.{tweet.full_text}')
#     iter+=1

def retrieve_conversation(tweet_id):
    tweet_thread= get_thread(api, tweet_id)
    tweet_thread_list = []
    tweet_thread_list = [tweet.full_text for tweet in tweet_thread]
    # print(tweet_thread_list)
    return tweet_thread_list
