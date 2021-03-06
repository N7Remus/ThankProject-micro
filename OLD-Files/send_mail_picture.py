import smtplib, ssl
import configparser
import argparse

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--im", type=str, required=True,
                    help="image_name")

config = configparser.ConfigParser()
config.read('config.ini')

smtp_server = "smtp.gmail.com"
port = 587  # For starttls

sender_email = config['EMAIL']['Email_user']
password = config['EMAIL']['Email_password']
#https://realpython.com/python-send-email/

# Create a secure SSL context
context = ssl.create_default_context()

# Try to log in to server and send email
try:
    server = smtplib.SMTP(smtp_server,port)
    server.ehlo() # Can be omitted
    server.starttls(context=context) # Secure the connection
    server.ehlo() # Can be omitted
    server.login(sender_email, password)
    # TODO: Send email here
    receiver_email = "remaigabor@gmail.com"
    message = """\
    Subject: Hi there

    This message is sent from Python."""


    # Send email here
    server.sendmail(sender_email, receiver_email, message)

except Exception as e:
    # Print any error messages to stdout
    print(e)
finally:
    server.quit()