# ROS and Amazon Web Service Integration (boto3)

## Step 1: Install the AWS python CLI

Follow the steps at the link below for linux:

[https://docs.aws.amazon.com/cli/latest/userguide/getting-started-install.htm](https://docs.aws.amazon.com/cli/latest/userguide/getting-started-install.html)l

## Step 2: Run the following commands on your VNC to install boto3

1. `pip install boto3`
2. `pip freeze`
3. check that `boto3` is actually there!
4. `python -m pip install --user boto3`

## Step 3: Integrating boto3 clients into your code

1. Create a file in your package to store your AWS credentials
2. Sign into AWS, and create an access key
3. Create an Access Policy for your services
4. Create a class that will hold your credentials, for example:
    
    ```python
    class Credentials:
        AWS_ACCESS_KEY_ID = ''
        SECRET_ACCESS_KEY = ''
        OWNER_ID = ''
    ```
    
    and any other information you may need to store for the service you choose to use.
    
5. Based on the service you’d like to use, create the appropriate client. If you’re using dynamoDB, it may look like this:
    
    ```python
    dynamodb = boto3.resource('dynamodb')
    ```
    
    Please refer to the documentation for of the service you’d like to use and the request format
    
    [https://boto3.amazonaws.com/v1/documentation/api/latest/reference/services/index.html](https://boto3.amazonaws.com/v1/documentation/api/latest/reference/services/index.html)
    
6. There are several helpful methods included with each service, here are some of dynamoDB’s methods for example:
    
    ```python
    delete_item()
    delete_table()
    put_item()
    query()
    update_item()
    update_table()
    ```
    
    Each method  will have a unique request, so be sure to check before implementing it into your code.
    
7. This can be used within your node, but keep in mind how many requests you are making if you place your request in a loop.