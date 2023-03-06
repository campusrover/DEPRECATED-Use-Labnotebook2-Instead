import tweepy
import os
import json
from dotenv import load_dotenv

load_dotenv()  # Load environment variables from .env file

consumer_key=os.getenv('Consumer_API_Key')
consumer_secret = os.getenv('Consumer_API_Secret')
access_token  = os.getenv('Access_Token')
access_token_secret  = os.getenv('Access_Token_Secret')

# # Authenticate with Twitter API using your own credentials
auth = tweepy.OAuth1UserHandler(
    consumer_key, consumer_secret,access_token,access_token_secret
)


# Initialize the API client
api = tweepy.API(auth)

# Set the tweet ID
tweet_id = "1631088599637819392"

# Get the tweet object
tweet = api.get_status(tweet_id,tweet_mode='extended',trim_user =True)
print(f'The tweet text is {tweet._json["full_text"]}')
# print(f'The tweet text is {tweet._json.keys()}')


# Get the conversation ID (aka thread ID) from the tweet object
# If there is a convesation id that means its not the first tweet in this thread

conversation_id = tweet.in_reply_to_status_id_str

# Print the conversation ID
print(conversation_id) #1630888649746153472
